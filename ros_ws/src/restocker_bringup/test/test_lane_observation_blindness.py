# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A blinded camera must not read as an empty lane, on the wire and not only in the arithmetic."""
# "No return inside the window" maps to available_depth_m = depth_m, the emptiest lane, so a
# failing sensor fails toward empty, which opens the capacity gate on a lane with no room. The
# producer must therefore say "I could not see" in a form no consumer can mistake for "empty";
# the depth number is not that form.
#
# Four depth buffers, one node, one lane, the same requested measurement each time:
#
#   * an empty lane rendered from the shipped station: coverage 1.0, status OK, 0.85 m free;
#   * the same lane with every sample blank: coverage 0.0, status INSUFFICIENT_COVERAGE, and the
#     identical 0.85 m free (coverage tells them apart, depth does not);
#   * no frames: nothing published and a distinct outcome (a dead sensor is not a blind one);
#   * a request that names no `not_before` (the destination acquire after a release sends one)
#     must be answered by the frame the camera produces next, never by the one already buffered
#     (Card 040, Milestone 10 §3).
#
# The node under test owns only the depth stream, so no simulator, planner or renderer is needed
# and the test runs in the Nix sandbox.

import math
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import numpy
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import LaneObservation
from restocker_interfaces.srv import AcquireLaneObservation
from sensor_msgs.msg import CameraInfo, Image

LANE_ID = "lane_03"
OPTICAL_FRAME = "wrist_camera_optical_frame"
OBSERVATION_TOPIC = "/perception/lane_observations"
DEPTH_TOPIC = "/test/wrist_depth_image"
CAMERA_INFO_TOPIC = "/test/wrist_camera_info"
ACQUIRE_SERVICE = "/test/acquire_lane_observation"

# The shipped wrist camera, from restocker_description/urdf/sensors.xacro.
WIDTH = 1280
HEIGHT = 720
HORIZONTAL_FOV = 1.48
FOCAL_PX = (WIDTH / 2.0) / math.tan(HORIZONTAL_FOV / 2.0)

# The shipped lane station, from restocker_perception/src/survey_stations.cpp: the camera stands
# behind the shelf's rear edge over the lane's centre line and looks at mid-lane depth. A lane
# frame is the shelf frame translated in X only, so in the lane's own frame the station is the
# same for every lane and the X offsets fall out.
CAMERA_SETBACK_M = 0.55
CAMERA_HEIGHT_M = 0.55
LANE_REAR_CLEARANCE_M = 0.01
LANE_USABLE_DEPTH_M = 0.85
LANE_HALF_SPAN_M = 0.1875
BED_DATUM_DEPTH_M = 0.84
LANE_INCLINE_RAD = math.radians(4.0)
FRONT_RETAINER_HEIGHT_M = 0.09

# The producer's configured coverage floor.
MINIMUM_COVERAGE = 0.90


def _station_pose_in_lane() -> numpy.ndarray:
    """Reproduce the shipped lane station as a 4x4 lane <- optical transform."""
    # The look-at the C++ station table builds: optical +Z is the boresight, +Y is image-down, and
    # image +X is held as close to the lane's +X as the boresight allows. Reproduced because there
    # is no robot to read it from.
    eye = numpy.array([0.0, -CAMERA_SETBACK_M, CAMERA_HEIGHT_M])
    target = numpy.array([0.0, LANE_REAR_CLEARANCE_M + (LANE_USABLE_DEPTH_M / 2.0), 0.0])
    forward = target - eye
    forward /= numpy.linalg.norm(forward)
    preferred_right = numpy.array([1.0, 0.0, 0.0])
    right = preferred_right - (preferred_right @ forward) * forward
    right /= numpy.linalg.norm(right)
    down = numpy.cross(forward, right)
    pose = numpy.eye(4)
    pose[:3, 0] = right
    pose[:3, 1] = down
    pose[:3, 2] = forward
    pose[:3, 3] = eye
    return pose


def _quaternion_from_matrix(rotation: numpy.ndarray) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a proper rotation matrix."""
    trace = rotation.trace()
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(numpy.argmax(numpy.diag(rotation)))
        other = [(axis + 1) % 3, (axis + 2) % 3]
        scale = (
            math.sqrt(
                1.0
                + rotation[axis, axis]
                - rotation[other[0], other[0]]
                - rotation[other[1], other[1]]
            )
            * 2.0
        )
        components = [0.0, 0.0, 0.0]
        components[axis] = 0.25 * scale
        components[other[0]] = (rotation[other[0], axis] + rotation[axis, other[0]]) / scale
        components[other[1]] = (rotation[other[1], axis] + rotation[axis, other[1]]) / scale
        w = (rotation[other[1], other[0]] - rotation[other[0], other[1]]) / scale
        x, y, z = components
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    return x / norm, y / norm, z / norm, w / norm


def _render_bed_and_retainer() -> numpy.ndarray:
    """Return the depth of the roller bed and the retainer, as this camera would see them."""
    # Two primitives: the bed plane z = (0.84 - y) * tan(4 degrees) and the retainer's rear face.
    pose = _station_pose_in_lane()
    origin = pose[:3, 3]
    columns, rows = numpy.meshgrid(numpy.arange(WIDTH), numpy.arange(HEIGHT))
    directions = (
        numpy.stack(
            [
                (columns - ((WIDTH - 1) / 2.0)) / FOCAL_PX,
                (rows - ((HEIGHT - 1) / 2.0)) / FOCAL_PX,
                numpy.ones_like(columns, dtype=float),
            ],
            axis=-1,
        )
        @ pose[:3, :3].T
    )

    slope = math.tan(LANE_INCLINE_RAD)
    normal = numpy.array([0.0, slope, 1.0])
    offset = BED_DATUM_DEPTH_M * slope
    denominator = directions @ normal
    with numpy.errstate(divide="ignore", invalid="ignore"):
        bed_t = (offset - (origin @ normal)) / denominator
    bed_hit = origin + (bed_t[..., None] * directions)
    bed_valid = (
        numpy.isfinite(bed_t)
        & (bed_t > 0.0)
        & (numpy.abs(bed_hit[..., 0]) <= LANE_HALF_SPAN_M)
        & (bed_hit[..., 1] >= -0.02)
        & (bed_hit[..., 1] <= BED_DATUM_DEPTH_M)
    )

    with numpy.errstate(divide="ignore", invalid="ignore"):
        retainer_t = (BED_DATUM_DEPTH_M - origin[1]) / directions[..., 1]
    retainer_hit = origin + (retainer_t[..., None] * directions)
    retainer_valid = (
        numpy.isfinite(retainer_t)
        & (retainer_t > 0.0)
        & (numpy.abs(retainer_hit[..., 0]) <= LANE_HALF_SPAN_M)
        & (retainer_hit[..., 2] >= 0.0)
        & (retainer_hit[..., 2] <= FRONT_RETAINER_HEIGHT_M)
    )

    depth = numpy.full((HEIGHT, WIDTH), numpy.nan, dtype=numpy.float32)
    depth[bed_valid] = bed_t[bed_valid]
    closer = retainer_valid & (~bed_valid | (retainer_t < numpy.nan_to_num(bed_t, nan=1e9)))
    depth[closer] = retainer_t[closer]
    return depth


def _render_empty_lane() -> numpy.ndarray:
    """Return the depth of the roller bed and the retainer, as this camera would see them."""
    return _render_bed_and_retainer()


# The shipped can (product_collision_catalog.yaml): a 0.066 m upright cylinder standing on the bed.
CAN_RADIUS_M = 0.033
CAN_HEIGHT_M = 0.122


def _render_lane_with_a_can(center_depth_m: float) -> numpy.ndarray:
    """
    Return the depth of the bed, the retainer and one can standing at this lane depth.

    `center_depth_m` is the can's axis in the lane frame, so its rearmost generator sits at
    center_depth_m - radius: 0.803 puts it at 0.770 (attempt 2's accepted reading, 0.7600 m free
    against a 0.01 m rear clearance) and 0.030 puts it at -0.003, proud of the entrance plane, so
    the lane reads full and obstructed as attempt 3 did.
    """
    depth = _render_bed_and_retainer()
    pose = _station_pose_in_lane()
    origin = pose[:3, 3]
    columns, rows = numpy.meshgrid(numpy.arange(WIDTH), numpy.arange(HEIGHT))
    directions = (
        numpy.stack(
            [
                (columns - ((WIDTH - 1) / 2.0)) / FOCAL_PX,
                (rows - ((HEIGHT - 1) / 2.0)) / FOCAL_PX,
                numpy.ones_like(columns, dtype=float),
            ],
            axis=-1,
        )
        @ pose[:3, :3].T
    )

    # Infinite cylinder about the lane's vertical axis at (0, center_depth_m); the nearer root is
    # the surface this camera sees, and the z bounds stand it on the bed.
    offset_y = origin[1] - center_depth_m
    quadratic_a = directions[..., 0] ** 2 + directions[..., 1] ** 2
    quadratic_b = 2.0 * (origin[0] * directions[..., 0] + offset_y * directions[..., 1])
    quadratic_c = origin[0] ** 2 + offset_y**2 - CAN_RADIUS_M**2
    discriminant = quadratic_b**2 - 4.0 * quadratic_a * quadratic_c
    with numpy.errstate(divide="ignore", invalid="ignore"):
        can_t = (-quadratic_b - numpy.sqrt(discriminant)) / (2.0 * quadratic_a)
    can_hit = origin + (can_t[..., None] * directions)
    base_z = (BED_DATUM_DEPTH_M - center_depth_m) * math.tan(LANE_INCLINE_RAD)
    can_valid = (
        numpy.isfinite(can_t)
        & (can_t > 0.0)
        & (can_hit[..., 2] >= base_z)
        & (can_hit[..., 2] <= base_z + CAN_HEIGHT_M)
    )
    closer = can_valid & (~numpy.isfinite(depth) | (can_t < numpy.nan_to_num(depth, nan=1e9)))
    depth[closer] = can_t[closer]
    return depth


def _render_lane_with_a_proud_can() -> numpy.ndarray:
    """Attempt 3's shape: the can's rearmost generator behind the entrance plane."""
    return _render_lane_with_a_can(0.030)


def _render_lane_with_a_seated_can() -> numpy.ndarray:
    """Attempt 2's shape: the can seated against the rail, 0.7600 m free, nothing protruding."""
    return _render_lane_with_a_can(0.803)


@pytest.mark.launch_test
def generate_test_description():
    """Start the lane observation node against a test-owned depth stream and one static frame."""
    workcell_geometry = str(
        Path(get_package_share_directory("restocker_description"))
        / "config"
        / "workcell_geometry.yaml"
    )
    lane_from_optical = _station_pose_in_lane()
    quaternion = _quaternion_from_matrix(lane_from_optical[:3, :3])
    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lane_camera_station",
        arguments=[
            "--x",
            str(lane_from_optical[0, 3]),
            "--y",
            str(lane_from_optical[1, 3]),
            "--z",
            str(lane_from_optical[2, 3]),
            "--qx",
            str(quaternion[0]),
            "--qy",
            str(quaternion[1]),
            "--qz",
            str(quaternion[2]),
            "--qw",
            str(quaternion[3]),
            "--frame-id",
            LANE_ID,
            "--child-frame-id",
            OPTICAL_FRAME,
        ],
    )
    lane_observation = Node(
        package="restocker_perception",
        executable="lane_observation_node",
        name="lane_observation",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "workcell_geometry": workcell_geometry,
                "depth_topic": DEPTH_TOPIC,
                "camera_info_topic": CAMERA_INFO_TOPIC,
                "lane_observation_topic": OBSERVATION_TOPIC,
                "acquire_service": ACQUIRE_SERVICE,
                "minimum_coverage": MINIMUM_COVERAGE,
                # Short: the "no frames" case waits it out.
                "acquisition_timeout_sec": 2.0,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [static_transform, lane_observation, launch_testing.actions.ReadyToTest()]
        ),
        {},
    )


class TestABlindedCameraIsNotAnEmptyLane(unittest.TestCase):
    """Empty and unseen carry the same depth, and only one of them carries proof."""

    @classmethod
    def setUpClass(cls):
        """Bring up a client node, its publishers, and the observation subscription."""
        rclpy.init()
        cls.node = rclpy.create_node("lane_observation_blindness_test")
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        cls.depth_publisher = cls.node.create_publisher(Image, DEPTH_TOPIC, sensor_qos)
        cls.info_publisher = cls.node.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, sensor_qos)
        cls.observations = []
        cls.node.create_subscription(
            LaneObservation,
            OBSERVATION_TOPIC,
            lambda message: cls.observations.append(message),
            20,
        )
        cls.client = cls.node.create_client(AcquireLaneObservation, ACQUIRE_SERVICE)
        assert cls.client.wait_for_service(timeout_sec=60.0), "acquisition service never appeared"
        cls.empty_lane_depth = _render_empty_lane()

    @classmethod
    def tearDownClass(cls):
        """Tear the client node down."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = OPTICAL_FRAME
        info.width = WIDTH
        info.height = HEIGHT
        info.k = [
            FOCAL_PX,
            0.0,
            (WIDTH - 1) / 2.0,
            0.0,
            FOCAL_PX,
            (HEIGHT - 1) / 2.0,
            0.0,
            0.0,
            1.0,
        ]
        return info

    def _depth_image(self, stamp, samples: numpy.ndarray) -> Image:
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = OPTICAL_FRAME
        image.width = WIDTH
        image.height = HEIGHT
        image.encoding = "32FC1"
        image.is_bigendian = 0
        image.step = WIDTH * 4
        image.data = samples.astype(numpy.float32).tobytes()
        return image

    def _acquire(self, samples, zero_not_before: bool = False):
        """Publish a frame, if there is one, then ask for a measurement of the lane."""
        # not_before is now, as a survey sets it when motion stops; otherwise each case would be
        # answered from the previous case's cached frame. zero_not_before drops that floor to the
        # request's own zero, which is what the coordinator's destination acquire sends.
        request = AcquireLaneObservation.Request()
        request.lane_id = LANE_ID
        if not zero_not_before:
            request.not_before = self.node.get_clock().now().to_msg()
        future = self.client.call_async(request)
        deadline = self.node.get_clock().now().nanoseconds + int(30e9)
        while not future.done() and self.node.get_clock().now().nanoseconds < deadline:
            if samples is not None:
                stamp = self.node.get_clock().now().to_msg()
                self.info_publisher.publish(self._camera_info(stamp))
                self.depth_publisher.publish(self._depth_image(stamp, samples))
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), "the acquisition service never answered")
        return future.result()

    def _publish_only(self, samples, seconds: float):
        """Keep the node's buffer fed with this scene, and answer nothing."""
        end = self.node.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.node.get_clock().now().nanoseconds < end:
            stamp = self.node.get_clock().now().to_msg()
            self.info_publisher.publish(self._camera_info(stamp))
            self.depth_publisher.publish(self._depth_image(stamp, samples))
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_a_lane_seen_to_be_empty_is_reported_empty_and_believed(self):
        """The bed is fully visible, so the answer stands."""
        response = self._acquire(self.empty_lane_depth)
        self.assertEqual(
            response.outcome,
            AcquireLaneObservation.Response.OUTCOME_PUBLISHED,
            response.detail,
        )
        observation = response.observation
        self.assertEqual(observation.status, LaneObservation.STATUS_OK, observation.status_detail)
        self.assertAlmostEqual(observation.available_depth_m, LANE_USABLE_DEPTH_M, places=6)
        self.assertGreaterEqual(observation.confidence, MINIMUM_COVERAGE)
        print(
            f"empty lane: {observation.available_depth_m:.4f} m free, "
            f"coverage {observation.confidence:.3f}"
        )

    def test_a_blinded_camera_reports_the_same_depth_and_is_refused(self):
        """Every sample blank: same depth as an empty lane, but refused."""
        blank = numpy.full((HEIGHT, WIDTH), numpy.nan, dtype=numpy.float32)
        response = self._acquire(blank)
        self.assertEqual(
            response.outcome,
            AcquireLaneObservation.Response.OUTCOME_PUBLISHED,
            response.detail,
        )
        observation = response.observation
        # Identical to the empty lane: the depth number alone is not evidence.
        self.assertAlmostEqual(observation.available_depth_m, LANE_USABLE_DEPTH_M, places=6)
        # Refused by both status and confidence.
        self.assertEqual(observation.status, LaneObservation.STATUS_INSUFFICIENT_COVERAGE)
        self.assertEqual(observation.confidence, 0.0)
        self.assertIn("coverage", observation.status_detail)
        print(f"blinded camera: {observation.available_depth_m:.4f} m free, refused")

    def test_a_camera_producing_no_frames_publishes_nothing_at_all(self):
        """A dead sensor is not a blind one: nothing is published rather than empty."""
        published_before = len(self.observations)
        response = self._acquire(None)
        self.assertEqual(
            response.outcome, AcquireLaneObservation.Response.OUTCOME_NO_FRAME, response.detail
        )
        # Nothing on the wire; the lane's existing evidence ages under the freshness gate.
        rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertEqual(len(self.observations), published_before)

    def test_a_zero_not_before_is_never_answered_from_the_buffered_frame(self):
        """
        The destination acquire names no instant, so it must get the *next* frame.

        Card 040's Terminal C: after a release the coordinator refreshes the destination lane
        through this service and passes no `not_before`, because it has just watched the arm stop
        at the retreat viewpoint. Answering that from the frame already in the buffer measures the
        instant of the previous consumer instead — attempt 3's post-release survey read the lane
        full and obstructed out of such a frame and the semantic commit was refused.
        """
        # Put attempt 3's shape — the can proud of the entrance, obstructed — in the buffer.
        buffered = self._acquire(_render_lane_with_a_proud_can())
        self.assertEqual(
            buffered.outcome,
            AcquireLaneObservation.Response.OUTCOME_PUBLISHED,
            buffered.detail,
        )
        self.assertLess(buffered.observation.available_depth_m, 1e-3)
        self.assertTrue(buffered.observation.obstructed)
        self._publish_only(_render_lane_with_a_proud_can(), 0.4)

        # Ask with the request's own zero, and give the camera nothing for a while: a floor of
        # zero must not be satisfied by the frame that is already there.
        request = AcquireLaneObservation.Request()
        request.lane_id = LANE_ID
        future = self.client.call_async(request)
        silent_until = self.node.get_clock().now().nanoseconds + int(1.0e9)
        while not future.done() and self.node.get_clock().now().nanoseconds < silent_until:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertFalse(
            future.done(),
            "the acquire was answered from the frame buffered before the request",
        )

        # The next frame the camera produces is attempt 2's accepted reading, and it is the one
        # that gets measured: healthy shape still accepted, obstructed shape still refused above.
        healthy = _render_lane_with_a_seated_can()
        deadline = self.node.get_clock().now().nanoseconds + int(2.0e9)
        while not future.done() and self.node.get_clock().now().nanoseconds < deadline:
            stamp = self.node.get_clock().now().to_msg()
            self.info_publisher.publish(self._camera_info(stamp))
            self.depth_publisher.publish(self._depth_image(stamp, healthy))
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), "the acquisition service never answered")
        response = future.result()
        self.assertEqual(
            response.outcome,
            AcquireLaneObservation.Response.OUTCOME_PUBLISHED,
            response.detail,
        )
        observation = response.observation
        self.assertEqual(observation.status, LaneObservation.STATUS_OK, observation.status_detail)
        self.assertAlmostEqual(observation.available_depth_m, 0.76, places=3)
        self.assertFalse(observation.obstructed)
        print(
            f"zero not_before: {observation.available_depth_m:.4f} m free, "
            f"obstructed={observation.obstructed}"
        )
