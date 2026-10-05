# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""RGB-D camera bring-up, publication-rate, and calibration acceptance test."""

import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import numpy
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from restocker_interfaces.msg import ObjectObservation
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import yaml

# The rates configured on the two sensors in restocker_description. Restated rather than parsed
# out of the description, because this test asserts that the whole path from the renderer through
# the bridge sustains them; reading the number back from the file that produced it would only
# prove the file is self-consistent.
OVERHEAD_RATE_HZ = 6.0
WRIST_RATE_HZ = 5.0

# How close the delivered rate has to sit to the configured one.
#
# The earlier bound (0.5x to 1.25x of nominal) was never derived, and the diagnosis that a
# software rasteriser cannot sustain the configured rate does not match measurement.
#
# Measured, subscribed reliably, over ten runs at loads from 55 to 105 with three other worktrees'
# simulators sharing the GPU: the overhead stream held 6.024 Hz and the wrist stream 5.000 Hz,
# with inter-frame simulation-time deltas of 0.165/0.166/0.167 s and 0.200/0.200/0.200 s. 6.024
# rather than 6.000 is quantisation: the sensor period is rounded to the 1 ms physics step, so
# 6 Hz becomes 0.166 s exactly.
#
# The renderer is nowhere near its limit at these rates. Asked for 10, 20 and 30 Hz it returned
# 10.000, 20.000 and 30.303 Hz with zero jitter, and the first dropped frames appear only at
# 45 Hz, 7.5x the overhead camera's configured rate.
#
# 10% is roughly twenty-five times the largest deviation observed. It is a band rather than a
# floor because a rate above nominal is a duplicated or double-bridged stream.
RATE_TOLERANCE = 0.10
# A single dropped frame must not fail this, so the cadence check uses the median gap rather than
# the maximum. Measured: the median gap equalled the minimum and the maximum to three decimals in
# every run at the configured rates.
PERIOD_TOLERANCE = 0.10

OVERHEAD_FRAME = "overhead_camera_optical_frame"
WRIST_FRAME = "wrist_camera_optical_frame"

# Every topic the bridge is expected to carry, with the frame its messages must be stamped with.
BRIDGED_TOPICS = {
    "/overhead_camera/image": (Image, OVERHEAD_FRAME, OVERHEAD_RATE_HZ),
    "/overhead_camera/depth_image": (Image, OVERHEAD_FRAME, OVERHEAD_RATE_HZ),
    "/overhead_camera/camera_info": (CameraInfo, OVERHEAD_FRAME, OVERHEAD_RATE_HZ),
    "/wrist_camera/image": (Image, WRIST_FRAME, WRIST_RATE_HZ),
    "/wrist_camera/depth_image": (Image, WRIST_FRAME, WRIST_RATE_HZ),
    "/wrist_camera/camera_info": (CameraInfo, WRIST_FRAME, WRIST_RATE_HZ),
}

CAN_OBJECT_ID = "sim:stock_can_01"
CAN_RADIUS_M = 0.033
CAN_HEIGHT_M = 0.122

# The projected position of a known object and the position it is drawn at differ slightly from
# rasterization alone, and the visible centroid of a cylinder is not exactly the projection of its
# centre because the camera sees one side of it. Measured disagreement is 1.06 px; 10 px is about
# ten times that, yet far below any fault that matters: 10 px at the can's 2.06 m range is 40 mm,
# more than its 33 mm radius, so a camera model wrong enough to fail this cannot be used to grasp.
CENTROID_TOLERANCE_PX = 10.0
# A depth pixel taken at the projected centre must land on the can's own surface, nearer than its
# centre by at most the circumscribing half-extent hypot(radius, height/2) = 69 mm. The measured
# sample is 58 mm nearer. A pixel that missed the can would hit the tray roughly 150 mm further
# along the ray, so this margin separates "on the object" from "on the background".
DEPTH_SURFACE_TOLERANCE_M = 0.02

# The can renders as roughly (225, 96, 79) under this world's ambient and key light: strongly red
# but not saturated, because ambient lifts the other two channels. Thresholds are set from that.
# The other two products are (75, 145, 193) and (65, 94, 84), and the search is confined to the
# tray anyway.
RED_MINIMUM = 100
RED_CHANNEL_RATIO = 1.8


@pytest.mark.launch_test
def generate_test_description():
    """Start the public simulation composition with its cameras and ground truth enabled."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "gui": "false",
            "cameras": "true",
            # Named explicitly although it is the default: this test asserts on the wrist stream,
            # so a future change to the argument's default must not silently disarm it.
            "wrist_camera": "true",
            "ground_truth": "true",
            # This test is about the sensor and its calibration, not about what is made of them.
            "perception": "false",
            # The launch sends ground truth to /perception/object_observations, since with
            # perception off that is the topic the world state ingests. This test reads
            # /perception/ground_truth/object_observations instead, because it measures the camera
            # model against poses the simulator knows to be true, not the camera's own estimates.
            # Without this line test_d waits out its 180 s for an observation published on the
            # other topic.
            "object_observation_topic": "/perception/ground_truth/object_observations",
            "controller_timeout": "30.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([simulation, launch_testing.actions.ReadyToTest()]),
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


class TestCameraRuntime(unittest.TestCase):
    """Assert the RGB-D sensors reach ROS correctly framed, on time, and correctly calibrated."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("camera_runtime_test")
        cls.latest = {}
        cls.stamps = {topic: [] for topic in BRIDGED_TOPICS}
        cls.delivered_best_effort = {topic: 0 for topic in BRIDGED_TOPICS}
        cls.observations = {}
        cls.subscriptions = []
        # Reliable, not qos_profile_sensor_data. These images are 2.1 to 3.7 MB each and under
        # best-effort the transport drops most of them: measured against one simulator at the
        # configured rates, the overhead camera_info arrived at 6.024 Hz while the overhead depth
        # image, produced by the same render pass, arrived at 0.900 Hz, and every gap in the image
        # stream was an exact multiple of the camera_info period. Subscribing to that topic alone
        # moved it only to 1.056 Hz, so the loss is in the transport, not this node's executor.
        # Reliability recovers all of it: all six topics arrive at exactly their configured rate.
        #
        # So this test proves the sensor produces frames on schedule and the bridge carries them.
        # It no longer proves that a best-effort consumer receives them; test_a's printout records
        # that number without asserting on it; depth_obstacle_node subscribes with
        # rclcpp::SensorDataQoS() and loses frames for the same reason. That is a transport
        # decision to take deliberately, not something a camera test should fail on.
        reliable = rclpy.qos.QoSProfile(depth=20, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        for topic, (message_type, _frame, _rate) in BRIDGED_TOPICS.items():
            cls.subscriptions.append(
                cls.node.create_subscription(
                    message_type,
                    topic,
                    cls._make_recorder(topic),
                    reliable,
                )
            )
            # A second, best-effort subscription to the same topic. It asserts nothing; test_a
            # prints how much of the stream a sensor-data consumer sees, so the cost of that QoS
            # stays visible.
            cls.subscriptions.append(
                cls.node.create_subscription(
                    message_type,
                    topic,
                    cls._make_best_effort_counter(topic),
                    qos_profile_sensor_data,
                )
            )
        # Ground truth, on its own topic. This test measures the camera model against poses the
        # simulator knows to be true; /perception/object_observations carries the camera's own
        # estimates, and measuring the camera against them would prove nothing.
        cls.subscriptions.append(
            cls.node.create_subscription(
                ObjectObservation,
                "/perception/ground_truth/object_observations",
                cls._on_observation,
                rclpy.qos.QoSProfile(depth=20),
            )
        )
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
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
    def _make_recorder(cls, topic):
        def record(message):
            cls.latest[topic] = message
            # The stamp is simulation time, the only clock that says whether the sensor kept its
            # configured rate. Wall-clock arrival would measure the machine's load.
            stamp = message.header.stamp
            cls.stamps[topic].append(stamp.sec + stamp.nanosec * 1e-9)

        return record

    @classmethod
    def _make_best_effort_counter(cls, topic):
        def count(_message):
            cls.delivered_best_effort[topic] += 1

        return count

    @classmethod
    def _on_observation(cls, observation):
        if observation.status == ObjectObservation.STATUS_OK:
            cls.observations[observation.source_object_id] = observation

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

    def test_a_every_bridged_topic_publishes_at_its_configured_rate(self):
        self._spin_until(
            lambda: all(self.stamps[topic] for topic in BRIDGED_TOPICS),
            180.0,
            "not every bridged camera topic published: missing "
            f"{sorted(topic for topic in BRIDGED_TOPICS if not self.stamps[topic])}",
        )
        for topic in BRIDGED_TOPICS:
            self.stamps[topic].clear()
            self.delivered_best_effort[topic] = 0
        self._spin_for(20.0)
        measured = {}
        periods = {}
        for topic in BRIDGED_TOPICS:
            stamps = sorted(self.stamps[topic])
            self.assertGreaterEqual(len(stamps), 5, f"{topic} published {len(stamps)} messages")
            span = stamps[-1] - stamps[0]
            self.assertGreater(span, 0.0, f"{topic} stamps did not advance")
            measured[topic] = (len(stamps) - 1) / span
            gaps = sorted(stamps[index + 1] - stamps[index] for index in range(len(stamps) - 1))
            periods[topic] = gaps[len(gaps) // 2]

        print("measured camera topic rates (Hz), reliable / best-effort delivered fraction:")
        for topic in sorted(BRIDGED_TOPICS):
            _type, _frame, nominal = BRIDGED_TOPICS[topic]
            received = len(self.stamps[topic])
            fraction = (self.delivered_best_effort[topic] / received) if received else 0.0
            print(
                f"  {topic}: {measured[topic]:.3f} Hz (nominal {nominal:.1f}), "
                f"median gap {periods[topic]:.4f} s, "
                f"best-effort delivered {self.delivered_best_effort[topic]}/{received} "
                f"({100 * fraction:.0f}%)"
            )

        for topic, (_type, _frame, nominal) in BRIDGED_TOPICS.items():
            # The sensor must be driven at its configured rate. Too slow is a renderer that cannot
            # keep up; too fast is a duplicated or double-bridged stream.
            self.assertGreater(
                measured[topic],
                (1.0 - RATE_TOLERANCE) * nominal,
                f"{topic} at {measured[topic]:.3f} Hz against nominal {nominal:.1f}",
            )
            self.assertLess(
                measured[topic],
                (1.0 + RATE_TOLERANCE) * nominal,
                f"{topic} at {measured[topic]:.3f} Hz against nominal {nominal:.1f}",
            )
            # The average rate can be right while the stream arrives in bursts, so the typical gap
            # is checked too: the median rather than the maximum, so one dropped frame in a twenty-
            # second window is not a failure.
            self.assertAlmostEqual(
                periods[topic],
                1.0 / nominal,
                delta=PERIOD_TOLERANCE / nominal,
                msg=f"{topic} median gap {periods[topic]:.4f} s against {1.0 / nominal:.4f} s",
            )

        # The three topics of one sensor come from a single render pass, so they cannot disagree
        # about how many frames there were. This catches a half-bridged or double-bridged stream
        # exactly, where a ratio against nominal only catches it loosely.
        for prefix in ("/overhead_camera", "/wrist_camera"):
            counts = {
                suffix: len(self.stamps[f"{prefix}/{suffix}"])
                for suffix in ("image", "depth_image", "camera_info")
            }
            self.assertLessEqual(
                max(counts.values()) - min(counts.values()),
                2,
                f"{prefix} streams disagree on frame count: {counts}",
            )

    def test_b_every_camera_message_is_stamped_with_a_resolvable_optical_frame(self):
        self._spin_until(
            lambda: all(topic in self.latest for topic in BRIDGED_TOPICS),
            180.0,
            "not every bridged camera topic published",
        )
        for topic, (_type, frame, _rate) in BRIDGED_TOPICS.items():
            self.assertEqual(self.latest[topic].header.frame_id, frame, f"{topic} frame_id")
        # The frame the images are stamped with must be reachable from the planning frame at the
        # stamp the image carries, or nothing downstream can put an observation into the world.
        # Both chains are asserted: the overhead camera resolves through the workcell branch, the
        # wrist camera through the moving arm.
        for frame, image_topic in (
            (OVERHEAD_FRAME, "/overhead_camera/image"),
            (WRIST_FRAME, "/wrist_camera/image"),
        ):
            stamp = self.latest[image_topic].header.stamp
            self._spin_until(
                lambda frame=frame, stamp=stamp: self.tf_buffer.can_transform(
                    "world", frame, Time.from_msg(stamp)
                ),
                60.0,
                f"tf could not resolve world <- {frame} at the image acquisition stamp",
            )

    def test_c_camera_info_carries_a_usable_pinhole_model(self):
        self._spin_until(
            lambda: all(topic in self.latest for topic in BRIDGED_TOPICS),
            180.0,
            "not every bridged camera topic published",
        )
        for prefix in ("/overhead_camera", "/wrist_camera"):
            info = self.latest[f"{prefix}/camera_info"]
            image = self.latest[f"{prefix}/image"]
            self.assertEqual((info.width, info.height), (image.width, image.height))
            fx, fy = info.k[0], info.k[4]
            self.assertGreater(fx, 0.0, f"{prefix} focal length")
            self.assertAlmostEqual(fx, fy, delta=1e-6, msg=f"{prefix} non-square pixels")
            self.assertAlmostEqual(info.k[2], info.width / 2.0, delta=0.5)
            self.assertAlmostEqual(info.k[5], info.height / 2.0, delta=0.5)
            self.assertEqual(info.k[1], 0.0)
            self.assertEqual(info.k[3], 0.0)
            self.assertEqual(info.k[8], 1.0)
            self.assertTrue(all(coefficient == 0.0 for coefficient in info.d))
            # An intrinsic matrix can be self-consistent and still describe an implausible lens.
            # Anything outside this band is a configuration mistake, not a camera.
            horizontal_fov = 2.0 * math.atan(info.width / (2.0 * fx))
            self.assertGreater(math.degrees(horizontal_fov), 40.0)
            self.assertLess(math.degrees(horizontal_fov), 120.0)
            print(
                f"{prefix}: {info.width}x{info.height} fx={fx:.2f} "
                f"hfov={math.degrees(horizontal_fov):.1f} deg"
            )

    def test_d_a_known_product_lands_where_the_calibration_projects_it(self):
        self._spin_until(
            lambda: (
                CAN_OBJECT_ID in self.observations
                and "/overhead_camera/image" in self.latest
                and "/overhead_camera/depth_image" in self.latest
                and "/overhead_camera/camera_info" in self.latest
                and self.tf_buffer.can_transform(OVERHEAD_FRAME, "world", Time())
                and self.tf_buffer.can_transform("world", "shelf", Time())
            ),
            180.0,
            "simulator ground truth and a bridged overhead frame did not both arrive",
        )
        # Let physics settle so the ground-truth pose the projection is measured against is the
        # pose the renderer drew, not the spawn pose before the first contact step.
        self._spin_for(5.0)

        info = self.latest["/overhead_camera/camera_info"]
        image = self.latest["/overhead_camera/image"]
        depth = self.latest["/overhead_camera/depth_image"]
        self.assertEqual(image.encoding, "rgb8")
        self.assertEqual(depth.encoding, "32FC1")

        observation = self.observations[CAN_OBJECT_ID]
        self.assertEqual(observation.header.frame_id, "world")
        position = observation.pose.pose.position
        can_world = numpy.array([position.x, position.y, position.z])

        # The transform is taken at the image's own acquisition stamp, not "latest". The overhead
        # chain is static so both answer the same today, but once this camera is on anything that
        # moves, "latest" silently becomes wrong.
        camera_from_world = self.tf_buffer.lookup_transform(
            OVERHEAD_FRAME, "world", Time.from_msg(image.header.stamp)
        )
        rotation, translation = _transform_parts(camera_from_world)
        can_camera = rotation @ can_world + translation
        self.assertGreater(can_camera[2], 0.0, "the can is behind the camera")
        projected_u, projected_v = _project(info, can_camera)
        self.assertTrue(0 <= projected_u < info.width and 0 <= projected_v < info.height)

        # Search only the stock tray for the can's colour. The bound is the surveyed tray volume
        # projected through the same model, so it is geometry rather than a window around the
        # answer, and it keeps the orange carriage, the only other warm-coloured body in the scene,
        # out of the measurement.
        min_u, min_v, max_u, max_v = self._tray_bounds(info)
        pixels = (
            numpy.frombuffer(bytes(image.data), dtype=numpy.uint8)
            .reshape(image.height, image.step // 1)[:, : image.width * 3]
            .reshape(image.height, image.width, 3)
            .astype(numpy.int32)
        )
        window = pixels[min_v:max_v, min_u:max_u]
        red, green, blue = window[:, :, 0], window[:, :, 1], window[:, :, 2]
        mask = (
            (red > RED_MINIMUM)
            & (red > RED_CHANNEL_RATIO * green)
            & (red > RED_CHANNEL_RATIO * blue)
        )
        self.assertGreater(int(mask.sum()), 30, "the can was not visible in the stock tray")
        rows, columns = numpy.nonzero(mask)
        centroid_u = float(columns.mean()) + min_u
        centroid_v = float(rows.mean()) + min_v
        centroid_error = math.hypot(centroid_u - projected_u, centroid_v - projected_v)

        # The depth pixel at the projected centre has to land on that same can. A wrong optical
        # frame, a wrong focal length or a transform read at the wrong instant all move this
        # sample off the object and onto the tray behind it.
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
            f"calibration check for {CAN_OBJECT_ID}:\n"
            f"  ground truth world      = ({can_world[0]:.4f}, {can_world[1]:.4f}, "
            f"{can_world[2]:.4f})\n"
            f"  ground truth in camera  = ({can_camera[0]:.4f}, {can_camera[1]:.4f}, "
            f"{can_camera[2]:.4f})\n"
            f"  projected pixel         = ({projected_u:.2f}, {projected_v:.2f})\n"
            f"  observed red centroid   = ({centroid_u:.2f}, {centroid_v:.2f})  "
            f"[{int(mask.sum())} px]\n"
            f"  centroid error          = {centroid_error:.2f} px\n"
            f"  depth at projected px   = {sample:.4f} m\n"
            f"  back-projected world    = ({surface_world[0]:.4f}, {surface_world[1]:.4f}, "
            f"{surface_world[2]:.4f})\n"
            f"  distance to can centre  = {surface_error:.4f} m"
        )
        self.assertLess(centroid_error, CENTROID_TOLERANCE_PX)
        # The sample sits on the surface facing the camera, so it is nearer than the centre by up
        # to the can's circumscribing half-extent and never further.
        self.assertLess(
            surface_error,
            math.hypot(CAN_RADIUS_M, CAN_HEIGHT_M / 2.0) + DEPTH_SURFACE_TOLERANCE_M,
        )
        self.assertLess(sample, can_camera[2])

    def _tray_bounds(self, info: CameraInfo) -> tuple[int, int, int, int]:
        """Project the surveyed stock tray volume into the image and return its pixel bounds."""
        volume = self.geometry["stock_tray"]["usable_volume"]
        centre = volume["center_xyz_m"]
        size = volume["size_xyz_m"]
        world_from_shelf = self.tf_buffer.lookup_transform("world", "shelf", Time())
        shelf_rotation, shelf_translation = _transform_parts(world_from_shelf)
        camera_from_world = self.tf_buffer.lookup_transform(OVERHEAD_FRAME, "world", Time())
        rotation, translation = _transform_parts(camera_from_world)
        us, vs = [], []
        for x in (centre[0] - size[0] / 2, centre[0] + size[0] / 2):
            for y in (centre[1] - size[1] / 2, centre[1] + size[1] / 2):
                for z in (centre[2] - size[2] / 2, centre[2] + size[2] / 2):
                    world = shelf_rotation @ numpy.array([x, y, z]) + shelf_translation
                    u, v = _project(info, rotation @ world + translation)
                    us.append(u)
                    vs.append(v)
        return (
            max(0, int(math.floor(min(us)))),
            max(0, int(math.floor(min(vs)))),
            min(info.width, int(math.ceil(max(us)))),
            min(info.height, int(math.ceil(max(vs)))),
        )
