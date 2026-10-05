# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The wrist camera goes where it is aimed, and sees what it should."""

# Measured here:
# 1. Every nominal survey station is commanded through the survey action and the achieved pose of
#    wrist_camera_optical_frame (read from TF, the whole kinematic chain) is compared with the
#    requested pose. The sides are independent: a mount applied on the wrong side would put every
#    achieved pose ~0.29 m off, which a unit test against a fake cannot notice.
# 2. Every station is checked for a collision-free IK solution and a plan from several seeds,
#    through move_group's own services. Failing stations are reported by name.
# 3. A known can is projected through the wrist camera's pinhole model at a commanded viewpoint,
#    the same check test_camera_runtime.py makes on the overhead camera (1.06 px).
#
# Separate from test_camera_runtime.py: that test asserts publication rates on a software renderer
# with no headroom; this one adds move_group and the projector. The projection helpers are
# duplicated because a launch test cannot import a sibling test module.

import math
from pathlib import Path
import statistics
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
import numpy
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from restocker_interfaces.action import SurveyViewpoint
from restocker_interfaces.msg import ObjectObservation
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import yaml

WRIST_FRAME = "wrist_camera_optical_frame"
PLANNING_FRAME = "world"
PLANNING_GROUP = "manipulator"
TOOL_LINK = "tool0"

# Nominal stations the survey server derives from the workcell geometry. Named, not posed:
# poses come back in each result, so this file does not restate their arithmetic.
LANE_STATIONS = ("lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06")
TRAY_STATIONS = ("tray_1", "tray_2")
STATIONS = LANE_STATIONS + TRAY_STATIONS

CAN_OBJECT_ID = "sim:stock_can_01"
CAN_RADIUS_M = 0.033
CAN_HEIGHT_M = 0.122

# Offset the projection check's viewpoint is commanded from, relative to the can: the same as a
# tray survey station's offset from the tray centre. Not the 0.18 m pre-grasp standoff (the camera
# is 0.110 m off the approach axis, putting the datum at 96% of the frame half-height), and not
# straight overhead (a solution exists only below world z 1.05, i.e. 0.42 m of standoff).
CONFIRM_SETBACK_M = 0.45
CONFIRM_HEIGHT_M = 0.555

# Achieved-versus-commanded tolerance for the optical frame, in metres and radians.
#
# The survey asks MoveIt for 0.006 m position and 0.010 rad per-axis orientation tolerance, so the
# geodesic orientation error may reach sqrt(3) * 0.010 = 0.0173 rad, and the error at the lens is
# 0.006 m plus the 0.146 m mount offset swung through that angle (0.0025 m): 0.0085 m. Measured
# over eight stations: translation 2.87 to 6.08 mm, rotation 5.40 to 13.68 mrad.
#
# 0.015 m and 0.035 rad are a little under twice the constructional bound and about 2.5 times the
# worst measured error. That costs 25 px of the tightest station's 47 px framing margin. The
# distribution is printed on every run.
ACHIEVED_TRANSLATION_TOLERANCE_M = 0.015
ACHIEVED_ROTATION_TOLERANCE_RAD = 0.035

# Ten times the 1.06 px the overhead camera measures on the same check (as in
# test_camera_runtime.py). At 0.55 m standoff 10 px is 7.9 mm, under the can's 33 mm radius.
CENTROID_TOLERANCE_PX = 10.0

# The can renders red but not saturated. From 0.55 m with the arm shading it from the key light,
# only the channel ratio is dependable (a shadow scales all channels together).
RED_MINIMUM = 60
RED_CHANNEL_RATIO = 1.8

# Seed configurations for the reachability sweep: rail coordinates with the arm at the SRDF
# `home` state. Seeding across the rail's travel varies the problem; the planning limit is +-1.68
# m.
SEED_RAIL_POSITIONS = (-1.5, -0.5, 0.0, 0.5, 1.5)


@pytest.mark.launch_test
def generate_test_description():
    """Start the planning composition with cameras and the wrist survey server."""
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
            "cameras": "true",
            "ground_truth": "true",
            # This test is about aiming the sensor, not about what is made of its images.
            "perception": "false",
            # The survey server and the coordinator are two clients of one move_group, so the
            # coordinator and everything downstream stay off. The planning-scene projector stays
            # on: the survey segment is gated on its authority.
            "task_coordinator": "false",
            "attachment_adapter": "false",
            "planning_smoke": "false",
            "planning_scene_projection": "true",
            "survey_viewpoint": "true",
            "controller_timeout": "60.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


def _rotation_from_quaternion(rotation) -> numpy.ndarray:
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return numpy.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _transform_parts(transform) -> tuple[numpy.ndarray, numpy.ndarray]:
    """Return (R, t) mapping a point in the source frame to the target frame."""
    translation = transform.transform.translation
    return (
        _rotation_from_quaternion(transform.transform.rotation),
        numpy.array([translation.x, translation.y, translation.z]),
    )


def _project(info: CameraInfo, point_camera: numpy.ndarray) -> tuple[float, float]:
    """Apply the published pinhole model to a point in the camera's optical frame."""
    fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
    return (
        fx * point_camera[0] / point_camera[2] + cx,
        fy * point_camera[1] / point_camera[2] + cy,
    )


def _look_at(eye, target, right, up):
    """
    Build an optical-frame rotation: +Z on the boresight, +Y image-down, +X image-right.

    Upright against `up` whenever the boresight is not parallel to it — the corrected
    production contract (Card 098, specs/camera-viewpoint-orientation.md). Until Card 098 this
    helper implemented the superseded hint-signed roll, which for a -Y-looking boresight
    commanded the fixture pose upside down. This test's projection check is TF-consistent and
    passes under either roll, so the fixture follows production instead of preserving old math.
    """
    boresight = numpy.array(target, dtype=float) - numpy.array(eye, dtype=float)
    boresight = boresight / numpy.linalg.norm(boresight)
    up_axis = numpy.array(up, dtype=float)
    up_axis = up_axis / numpy.linalg.norm(up_axis)
    up_perp = up_axis - up_axis.dot(boresight) * boresight
    if numpy.linalg.norm(up_perp) >= 1e-6:
        axis_x = numpy.cross(-up_perp / numpy.linalg.norm(up_perp), boresight)
    else:
        # Straight up/down: every roll is upright, so the right hint pins one (as production).
        axis_x = numpy.array(right, dtype=float)
        axis_x = axis_x - axis_x.dot(boresight) * boresight
        axis_x = axis_x / numpy.linalg.norm(axis_x)
    return numpy.column_stack([axis_x, numpy.cross(boresight, axis_x), boresight])


def _quaternion_from_matrix(matrix) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation matrix, by the branch with the largest divisor."""
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        return (
            0.25 * scale,
            (matrix[0, 1] + matrix[1, 0]) / scale,
            (matrix[0, 2] + matrix[2, 0]) / scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
        )
    if matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        return (
            (matrix[0, 1] + matrix[1, 0]) / scale,
            0.25 * scale,
            (matrix[1, 2] + matrix[2, 1]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
        )
    scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
    return (
        (matrix[0, 2] + matrix[2, 0]) / scale,
        (matrix[1, 2] + matrix[2, 1]) / scale,
        0.25 * scale,
        (matrix[1, 0] - matrix[0, 1]) / scale,
    )


def _pose_parts(pose: Pose) -> tuple[numpy.ndarray, numpy.ndarray]:
    """Return (R, t) for a pose, read as the transform from the posed body into its frame."""
    return (
        _rotation_from_quaternion(pose.orientation),
        numpy.array([pose.position.x, pose.position.y, pose.position.z]),
    )


class TestWristViewpointRuntime(unittest.TestCase):
    """Assert the wrist camera goes where it is aimed, and sees what the model says it sees."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("wrist_viewpoint_test")
        cls.latest = {}
        cls.stamps = {}
        cls.observations = {}
        cls.joint_state = None
        cls.results = {}
        cls.subscriptions = [
            cls.node.create_subscription(
                Image,
                "/wrist_camera/image",
                cls._recorder("/wrist_camera/image"),
                qos_profile_sensor_data,
            ),
            cls.node.create_subscription(
                Image,
                "/wrist_camera/depth_image",
                cls._recorder("/wrist_camera/depth_image"),
                qos_profile_sensor_data,
            ),
            cls.node.create_subscription(
                CameraInfo,
                "/wrist_camera/camera_info",
                cls._recorder("/wrist_camera/camera_info"),
                qos_profile_sensor_data,
            ),
            # With perception:=false this topic carries simulator ground truth and is the one the
            # world state ingests. Moving ground truth elsewhere would leave the world state empty,
            # so the projector would certify nothing and the motion port's authority gate would
            # refuse every survey segment.
            cls.node.create_subscription(
                ObjectObservation,
                "/perception/object_observations",
                cls._on_observation,
                rclpy.qos.QoSProfile(depth=20),
            ),
            cls.node.create_subscription(
                JointState, "/joint_states", cls._on_joint_state, qos_profile_sensor_data
            ),
        ]
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.survey = ActionClient(cls.node, SurveyViewpoint, "/survey_viewpoint")
        cls.ik_client = cls.node.create_client(GetPositionIK, "/compute_ik")
        cls.plan_client = cls.node.create_client(GetMotionPlan, "/plan_kinematic_path")
        geometry_file = (
            Path(get_package_share_directory("restocker_description"))
            / "config"
            / "workcell_geometry.yaml"
        )
        cls.geometry = yaml.safe_load(geometry_file.read_text(encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _recorder(cls, topic):
        def record(message):
            cls.latest[topic] = message
            # Simulation time is the clock that says when the sensor acquired; wall arrival would
            # measure machine load.
            stamp = message.header.stamp
            cls.stamps.setdefault(topic, []).append(stamp.sec + stamp.nanosec * 1e-9)

        return record

    @classmethod
    def _on_observation(cls, observation):
        if observation.status == ObjectObservation.STATUS_OK:
            cls.observations[observation.source_object_id] = observation

    @classmethod
    def _on_joint_state(cls, state):
        cls.joint_state = state

    def _spin_until(self, predicate, timeout_sec, failure):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(failure)

    def _spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _await(self, future, timeout_sec, failure):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        self.fail(failure)
        return None

    def _survey(self, goal: SurveyViewpoint.Goal, timeout_sec: float = 180.0):
        """Send one survey goal and return its result, spinning this node throughout."""
        send = self.survey.send_goal_async(goal)
        handle = self._await(send, 30.0, f"survey goal {goal.station or goal.label} not accepted")
        self.assertTrue(handle.accepted, f"survey server rejected {goal.station or goal.label}")
        result = self._await(
            handle.get_result_async(),
            timeout_sec,
            f"survey goal {goal.station or goal.label} produced no result",
        )
        return result.result

    def _await_backends(self):
        self._spin_until(
            lambda: (
                self.survey.server_is_ready()
                and self.ik_client.service_is_ready()
                and self.plan_client.service_is_ready()
                and self.joint_state is not None
                and CAN_OBJECT_ID in self.observations
                and "/wrist_camera/camera_info" in self.latest
                and self.tf_buffer.can_transform(PLANNING_FRAME, WRIST_FRAME, Time())
            ),
            300.0,
            "the survey server, move_group services, ground truth and the wrist camera did not "
            "all come up",
        )

    # -- the eight nominal stations ---------------------------------------------------------

    def test_a_every_nominal_station_is_commanded_and_the_camera_lands_where_it_was_aimed(self):
        self._await_backends()
        # Let the simulator settle so the first plan is not made against a scene still dropping
        # products onto the tray.
        self._spin_for(5.0)

        unreachable = []
        errors = {}
        print("\ncommanded survey stations:")
        for station in STATIONS:
            goal = SurveyViewpoint.Goal()
            goal.station = station
            goal.label = station
            result = self._survey(goal)
            type(self).results[station] = result
            arrived = result.outcome == SurveyViewpoint.Result.OUTCOME_ARRIVED
            commanded = result.commanded_camera_optical_pose.pose.position
            print(
                f"  {station}: outcome={result.outcome} "
                f"({'arrived' if arrived else 'NOT REACHED'}) "
                f"camera=({commanded.x:.3f}, {commanded.y:.3f}, {commanded.z:.3f}) "
                f"tool=({result.commanded_tool0_pose.pose.position.x:.3f}, "
                f"{result.commanded_tool0_pose.pose.position.y:.3f}, "
                f"{result.commanded_tool0_pose.pose.position.z:.3f}) "
                f"error={result.achieved_translation_error_m * 1000.0:.1f} mm / "
                f"{result.achieved_rotation_error_rad * 1000.0:.1f} mrad "
                f"detail={result.detail}"
            )
            # A viewpoint that cannot be turned into a tool goal is an arithmetic failure, never
            # acceptable.
            self.assertNotEqual(
                result.outcome,
                SurveyViewpoint.Result.OUTCOME_VIEWPOINT_UNRESOLVED,
                f"{station}: the mount transform could not be applied: {result.detail}",
            )
            self.assertNotEqual(
                result.outcome,
                SurveyViewpoint.Result.OUTCOME_INVALID_REQUEST,
                f"{station}: {result.detail}",
            )
            if not arrived:
                unreachable.append(f"{station} ({result.detail})")
                continue
            self.assertTrue(
                result.achieved_pose_measured,
                f"{station} arrived but its achieved camera pose was never measured",
            )
            errors[station] = (
                result.achieved_translation_error_m,
                result.achieved_rotation_error_rad,
            )

        translation_errors = [pair[0] for pair in errors.values()]
        rotation_errors = [pair[1] for pair in errors.values()]
        if translation_errors:
            print(
                "achieved-versus-commanded optical pose error over "
                f"{len(translation_errors)} stations: "
                f"translation min {min(translation_errors) * 1000.0:.2f} mm, "
                f"median {statistics.median(translation_errors) * 1000.0:.2f} mm, "
                f"max {max(translation_errors) * 1000.0:.2f} mm; "
                f"rotation min {min(rotation_errors) * 1000.0:.2f} mrad, "
                f"median {statistics.median(rotation_errors) * 1000.0:.2f} mrad, "
                f"max {max(rotation_errors) * 1000.0:.2f} mrad"
            )

        self.assertFalse(
            unreachable,
            "these nominal survey stations were not reached: " + "; ".join(unreachable),
        )
        for station, (translation, rotation) in errors.items():
            self.assertLess(
                translation,
                ACHIEVED_TRANSLATION_TOLERANCE_M,
                f"{station} achieved optical pose is {translation * 1000.0:.1f} mm from the "
                "command",
            )
            self.assertLess(
                rotation,
                ACHIEVED_ROTATION_TOLERANCE_RAD,
                f"{station} achieved optical attitude is {rotation * 1000.0:.1f} mrad from the "
                "command",
            )

    # -- reachability from a set of seeds ---------------------------------------------------

    def _seed_states(self) -> list[tuple[str, RobotState]]:
        """Build seed robot states across the rail's travel, with the arm at its home state."""
        assert self.joint_state is not None
        seeds = []
        for rail in SEED_RAIL_POSITIONS:
            state = RobotState()
            state.joint_state.name = list(self.joint_state.name)
            positions = []
            for name in state.joint_state.name:
                if name == "rail_joint":
                    positions.append(rail)
                else:
                    positions.append(0.0)
            state.joint_state.position = positions
            state.is_diff = False
            seeds.append((f"rail={rail:+.1f}", state))
        return seeds

    @staticmethod
    def _pose_goal(pose: PoseStamped) -> Constraints:
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = PLANNING_FRAME
        position.link_name = TOOL_LINK
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.010]
        position.constraint_region.primitives.append(primitive)
        position.constraint_region.primitive_poses.append(pose.pose)
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = PLANNING_FRAME
        orientation.link_name = TOOL_LINK
        orientation.orientation = pose.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.020
        orientation.absolute_y_axis_tolerance = 0.020
        orientation.absolute_z_axis_tolerance = 0.020
        orientation.weight = 1.0
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return constraints

    def test_b_every_nominal_station_has_a_collision_free_solution_from_a_set_of_seeds(self):
        self._await_backends()
        self.assertTrue(
            self.results, "the station sweep did not run, so there are no tool poses to solve for"
        )
        seeds = self._seed_states()
        ik_failures = []
        plan_failures = []
        print("\nreachability of every nominal station, per seed:")
        for station in STATIONS:
            result = self.results.get(station)
            self.assertIsNotNone(result, f"{station} produced no survey result")
            tool_pose = result.commanded_tool0_pose
            self.assertEqual(
                tool_pose.header.frame_id,
                PLANNING_FRAME,
                f"{station} tool goal is not in the planning frame",
            )
            solved = []
            planned = []
            for label, seed in seeds:
                ik_request = GetPositionIK.Request()
                ik_request.ik_request.group_name = PLANNING_GROUP
                ik_request.ik_request.ik_link_name = TOOL_LINK
                ik_request.ik_request.robot_state = seed
                ik_request.ik_request.pose_stamped = tool_pose
                # A solution in collision is not a solution.
                ik_request.ik_request.avoid_collisions = True
                ik_request.ik_request.timeout.sec = 2
                answer = self._await(
                    self.ik_client.call_async(ik_request),
                    30.0,
                    f"/compute_ik did not answer for {station} at {label}",
                )
                if answer.error_code.val == 1:
                    solved.append(label)

                plan_request = GetMotionPlan.Request()
                request = MotionPlanRequest()
                request.group_name = PLANNING_GROUP
                request.start_state = seed
                request.goal_constraints.append(self._pose_goal(tool_pose))
                request.allowed_planning_time = 5.0
                request.num_planning_attempts = 4
                request.max_velocity_scaling_factor = 0.2
                request.max_acceleration_scaling_factor = 0.2
                plan_request.motion_plan_request = request
                plan_answer = self._await(
                    self.plan_client.call_async(plan_request),
                    60.0,
                    f"/plan_kinematic_path did not answer for {station} at {label}",
                )
                if plan_answer.motion_plan_response.error_code.val == 1:
                    planned.append(label)

            print(
                f"  {station}: IK from {len(solved)}/{len(seeds)} seeds {solved}, "
                f"plan from {len(planned)}/{len(seeds)} seeds {planned}"
            )
            if not solved:
                ik_failures.append(station)
            if not planned:
                plan_failures.append(station)

        self.assertFalse(
            ik_failures,
            "these nominal survey stations have no collision-free IK solution from any seed: "
            + ", ".join(ik_failures),
        )
        self.assertFalse(
            plan_failures,
            "these nominal survey stations cannot be planned to from any seed: "
            + ", ".join(plan_failures),
        )

    # -- the projection check ---------------------------------------------------------------

    def test_c_a_known_product_lands_where_the_wrist_calibration_projects_it(self):
        self._await_backends()
        observation = self.observations[CAN_OBJECT_ID]
        self.assertEqual(observation.header.frame_id, PLANNING_FRAME)
        position = observation.pose.pose.position
        can_world = numpy.array([position.x, position.y, position.z])

        # Viewpoint offset from the can as a tray station is from the tray centre, boresight on
        # the can, UPRIGHT against world +Z (Card 098's corrected look-at contract; world +X is
        # only the degenerate hint now). Explicit-pose path, so the optical-frame convention is
        # restated here and the projection check catches a mistake.
        eye = can_world + numpy.array([0.0, CONFIRM_SETBACK_M, CONFIRM_HEIGHT_M])
        rotation = _look_at(eye, can_world, [1.0, 0.0, 0.0], [0.0, 0.0, 1.0])
        quaternion = _quaternion_from_matrix(rotation)
        goal = SurveyViewpoint.Goal()
        goal.label = "confirm the stock can"
        goal.camera_optical_pose.header.frame_id = PLANNING_FRAME
        goal.camera_optical_pose.pose.position.x = float(eye[0])
        goal.camera_optical_pose.pose.position.y = float(eye[1])
        goal.camera_optical_pose.pose.position.z = float(eye[2])
        goal.camera_optical_pose.pose.orientation.x = quaternion[0]
        goal.camera_optical_pose.pose.orientation.y = quaternion[1]
        goal.camera_optical_pose.pose.orientation.z = quaternion[2]
        goal.camera_optical_pose.pose.orientation.w = quaternion[3]
        result = self._survey(goal)
        print(
            f"\nconfirmation viewpoint over {CAN_OBJECT_ID}: outcome={result.outcome} "
            f"error={result.achieved_translation_error_m * 1000.0:.1f} mm / "
            f"{result.achieved_rotation_error_rad * 1000.0:.1f} mrad detail={result.detail}"
        )
        self.assertEqual(
            result.outcome,
            SurveyViewpoint.Result.OUTCOME_ARRIVED,
            f"the confirmation viewpoint was not reached: {result.detail}",
        )
        self.assertLess(result.achieved_translation_error_m, ACHIEVED_TRANSLATION_TOLERANCE_M)
        self.assertLess(result.achieved_rotation_error_rad, ACHIEVED_ROTATION_TOLERANCE_RAD)

        # Take a frame acquired strictly after the arm stopped; an earlier frame would pass without
        # proving anything about the commanded viewpoint.
        arrival_stamp = result.achieved_camera_optical_pose.header.stamp
        arrival = arrival_stamp.sec + arrival_stamp.nanosec * 1e-9

        # Wait for whole frames, not wall-clock time: the software renderer runs below the
        # configured 5 Hz (as low as 2.5 Hz). Three frames after arrival: the first is what a
        # survey would acquire, the third was rendered with the arm at rest.
        def frames_after(topic):
            return [stamp for stamp in self.stamps.get(topic, []) if stamp > arrival]

        self._spin_until(
            lambda: (
                len(frames_after("/wrist_camera/image")) >= 3
                and len(frames_after("/wrist_camera/depth_image")) >= 3
            ),
            300.0,
            "fewer than three wrist camera frames arrived after the survey reached its "
            "viewpoint; the renderer is not delivering this stream",
        )

        # Measures how long after the arm stops the first usable frame exists (dwell time per
        # viewpoint is one frame period).
        image_stamps = frames_after("/wrist_camera/image")
        acquisition_latency = image_stamps[0] - arrival
        all_stamps = sorted(self.stamps.get("/wrist_camera/image", []))
        measured_rate = (
            (len(all_stamps) - 1) / (all_stamps[-1] - all_stamps[0])
            if len(all_stamps) > 1 and all_stamps[-1] > all_stamps[0]
            else float("nan")
        )
        print(
            f"wrist acquisition after arriving at a viewpoint: first frame "
            f"{acquisition_latency:.3f} s of simulated time after the arm stopped; "
            f"third frame {image_stamps[2] - arrival:.3f} s; "
            f"stream measured at {measured_rate:.2f} Hz over {len(all_stamps)} frames "
            f"against a configured 5.0 Hz"
        )

        info = self.latest["/wrist_camera/camera_info"]
        image = self.latest["/wrist_camera/image"]
        depth = self.latest["/wrist_camera/depth_image"]
        self.assertEqual(image.encoding, "rgb8")
        self.assertEqual(depth.encoding, "32FC1")
        self.assertEqual((info.width, info.height), (image.width, image.height))

        # The transform is taken at the image's acquisition stamp, not "latest": the wrist chain
        # runs through the moving arm. The stamp can be ~10 ms ahead of the newest transform (the
        # camera renders between two robot_state_publisher samples), so wait for TF to reach the
        # stamp; never fall back to "latest".
        acquisition = Time.from_msg(image.header.stamp)
        self._spin_until(
            lambda: self.tf_buffer.can_transform(WRIST_FRAME, PLANNING_FRAME, acquisition),
            30.0,
            "tf never reached the wrist image's acquisition stamp",
        )
        camera_from_world = self.tf_buffer.lookup_transform(
            WRIST_FRAME, PLANNING_FRAME, acquisition
        )
        rotation, translation = _transform_parts(camera_from_world)
        can_camera = rotation @ can_world + translation
        self.assertGreater(can_camera[2], 0.0, "the can is behind the wrist camera")
        projected_u, projected_v = _project(info, can_camera)
        self.assertTrue(
            0 <= projected_u < info.width and 0 <= projected_v < info.height,
            f"the can projects outside the wrist image at ({projected_u:.1f}, {projected_v:.1f})",
        )

        min_u, min_v, max_u, max_v = self._tray_bounds(info, rotation, translation)
        pixels = (
            numpy.frombuffer(bytes(image.data), dtype=numpy.uint8)
            .reshape(image.height, image.step // 1)[:, : image.width * 3]
            .reshape(image.height, image.width, 3)
            .astype(numpy.int32)
        )
        window = pixels[min_v:max_v, min_u:max_u]
        self.assertGreater(window.size, 0, "the stock tray projects to an empty window")
        red, green, blue = window[:, :, 0], window[:, :, 1], window[:, :, 2]
        mask = (
            (red > RED_MINIMUM)
            & (red > RED_CHANNEL_RATIO * green)
            & (red > RED_CHANNEL_RATIO * blue)
        )
        self.assertGreater(int(mask.sum()), 200, "the can was not visible from the wrist camera")
        rows, columns = numpy.nonzero(mask)
        centroid_u = float(columns.mean()) + min_u
        centroid_v = float(rows.mean()) + min_v
        centroid_error = math.hypot(centroid_u - projected_u, centroid_v - projected_v)

        depth_map = numpy.frombuffer(bytes(depth.data), dtype=numpy.float32).reshape(
            depth.height, depth.step // 4
        )[:, : depth.width]
        sample = float(depth_map[int(round(projected_v)), int(round(projected_u))])
        self.assertTrue(math.isfinite(sample) and sample > 0.0, "no depth return on the can")
        surface_camera = numpy.array(
            [
                (projected_u - info.k[2]) * sample / info.k[0],
                (projected_v - info.k[5]) * sample / info.k[4],
                sample,
            ]
        )
        surface_world = rotation.T @ (surface_camera - translation)
        surface_error = float(numpy.linalg.norm(surface_world - can_world))

        print(
            f"wrist calibration check for {CAN_OBJECT_ID}:\n"
            f"  ground truth world      = ({can_world[0]:.4f}, {can_world[1]:.4f}, "
            f"{can_world[2]:.4f})\n"
            f"  ground truth in camera  = ({can_camera[0]:.4f}, {can_camera[1]:.4f}, "
            f"{can_camera[2]:.4f})\n"
            f"  projected pixel         = ({projected_u:.2f}, {projected_v:.2f}) "
            f"of {info.width}x{info.height}\n"
            f"  observed red centroid   = ({centroid_u:.2f}, {centroid_v:.2f})  "
            f"[{int(mask.sum())} px]\n"
            f"  centroid error          = {centroid_error:.2f} px\n"
            f"  depth at projected px   = {sample:.4f} m\n"
            f"  distance to can centre  = {surface_error:.4f} m"
        )
        self.assertLess(centroid_error, CENTROID_TOLERANCE_PX)
        self.assertLess(surface_error, math.hypot(CAN_RADIUS_M, CAN_HEIGHT_M / 2.0) + 0.02)
        self.assertLess(sample, can_camera[2])

    def _tray_bounds(
        self, info: CameraInfo, rotation: numpy.ndarray, translation: numpy.ndarray
    ) -> tuple[int, int, int, int]:
        """Project the surveyed stock tray volume and return its pixel bounds in this image."""
        volume = self.geometry["stock_tray"]["usable_volume"]
        centre = volume["center_xyz_m"]
        size = volume["size_xyz_m"]
        world_from_shelf = self.tf_buffer.lookup_transform(PLANNING_FRAME, "shelf", Time())
        shelf_rotation, shelf_translation = _transform_parts(world_from_shelf)
        us, vs = [], []
        for x in (centre[0] - size[0] / 2, centre[0] + size[0] / 2):
            for y in (centre[1] - size[1] / 2, centre[1] + size[1] / 2):
                for z in (centre[2] - size[2] / 2, centre[2] + size[2] / 2):
                    world = shelf_rotation @ numpy.array([x, y, z]) + shelf_translation
                    camera = rotation @ world + translation
                    if camera[2] <= 0.0:
                        continue
                    u, v = _project(info, camera)
                    us.append(u)
                    vs.append(v)
        self.assertTrue(us, "no corner of the stock tray is in front of the wrist camera")
        return (
            max(0, int(math.floor(min(us)))),
            max(0, int(math.floor(min(vs)))),
            min(info.width, int(math.ceil(max(us)))),
            min(info.height, int(math.ceil(max(vs)))),
        )


@launch_testing.post_shutdown_test()
class TestSurveyViewpointNodeDidNotCrash(unittest.TestCase):
    """Fail the run when `survey_viewpoint_node` died on a fault rather than exiting."""

    def test_survey_viewpoint_node_exited_cleanly(self, proc_info):
        """Assert `survey_viewpoint_node` was not killed by SIGSEGV, SIGABRT, SIGBUS or SIGILL."""
        fatal = {-4: "SIGILL", -6: "SIGABRT", -7: "SIGBUS", -11: "SIGSEGV"}
        crashed = [
            f"{info.process_name} died on {fatal[info.returncode]}"
            for info in proc_info
            if "survey_viewpoint_node" in info.process_name and info.returncode in fatal
        ]
        self.assertEqual(crashed, [], f"survey_viewpoint_node crashed: {crashed}")
