# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Drive the depth obstacle node with a synthetic depth image and read back its obstacle."""
# Extraction itself is tested in `restocker_perception/test/test_depth_obstacle_extraction.cpp`.
# This covers the wiring around it: parameters, the 32FC1 payload, transforms resolved at the
# image stamp, and the message the projector consumes. It needs no camera or simulator.

import array
import math
import struct
import time
import unittest

from geometry_msgs.msg import TransformStamped
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.duration import Duration
from restocker_interfaces.msg import ObstacleObservation
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

WIDTH = 64
HEIGHT = 48
FOCAL = 64.0
CAMERA_HEIGHT_M = 2.0
# Range to the synthetic surface. Sample spacing there (RANGE / FOCAL = 0.0125 m) is finer than
# the 0.02 m voxel, which keeps the patch one connected cluster.
SURFACE_RANGE_M = 0.8
PATCH_COLUMNS = range(24, 40)
PATCH_ROWS = range(16, 32)
OPTICAL_FRAME = "synthetic_camera_optical_frame"
OBSTACLE_TOPIC = "/perception/obstacle_observations"


@pytest.mark.launch_test
def generate_test_description():
    """Start only the depth obstacle node, with the probe volumes this test provides."""
    detector = Node(
        package="restocker_perception",
        executable="depth_obstacle_node",
        name="depth_obstacle_node_runtime_test",
        output="screen",
        parameters=[
            {
                "planning_frame": "world",
                "depth_topic": "/synthetic_camera/depth_image",
                "camera_info_topic": "/synthetic_camera/camera_info",
                "output_topic": OBSTACLE_TOPIC,
                "pixel_stride": 1,
                "min_depth_m": 0.1,
                "max_depth_m": 6.0,
                "voxel_size_m": 0.02,
                "min_cluster_voxels": 12,
                "volume_of_interest_min_xyz_m": [-1.0, -1.0, 0.20],
                "volume_of_interest_max_xyz_m": [1.0, 1.0, 1.60],
                # One self-filter volume parked away from the surface, so the frame lookup and
                # stillness probe run without removing the obstacle. A zero-length segment makes
                # it a sphere at the frame.
                # Raised to 2.0 s so the per-frame transform budget test below has room to tell
                # one shared wait apart from one wait per lookup (2 waits here, 23 in the shipped
                # default self_filter_frames list).
                "transform_timeout_sec": 2.0,
                # Six identical probe volumes, all parked away from the surface, so the budget
                # test can feed their transforms one frame at a time; one sphere at the same
                # place would only ever cost one wait.
                "self_filter_frames": [f"synthetic_probe_link_{i}" for i in range(6)],
                "self_filter_segments_xyz_m": [0.0] * 36,
                "self_filter_radii_m": [0.01] * 6,
            }
        ],
    )
    return launch.LaunchDescription([detector, launch_testing.actions.ReadyToTest()])


class TestDepthObstacleNodeRuntime(unittest.TestCase):
    """Require one depth image to become one correctly placed obstacle."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("depth_obstacle_node_runtime_test_client")
        cls.static_tf = tf2_ros.StaticTransformBroadcaster(cls.node)
        # The probe frame is published dynamically so a stamp outside its range can be produced
        # on purpose: the budget test needs a frame that does *not* answer at every time, which a
        # static transform always would.
        cls.dynamic_tf = tf2_ros.TransformBroadcaster(cls.node)
        cls.info_publisher = cls.node.create_publisher(
            CameraInfo, "/synthetic_camera/camera_info", 10
        )
        cls.image_publisher = cls.node.create_publisher(Image, "/synthetic_camera/depth_image", 10)
        cls.observations = []
        cls.node.create_subscription(
            ObstacleObservation, OBSTACLE_TOPIC, cls.observations.append, 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            result = predicate()
            if result:
                return result
        self.fail(f"timed out waiting for {description}")

    def _broadcast_frames(self):
        """Publish the camera pose statically and the self-filter probe dynamically."""
        # The camera looks straight down from two metres: optical +z is world -z and optical +y is
        # world -y (a half turn about x). The camera transform is static, so it answers at any
        # stamp including the deliberate one the budget test uses.
        camera = TransformStamped()
        camera.header.stamp = self.node.get_clock().now().to_msg()
        camera.header.frame_id = "world"
        camera.child_frame_id = OPTICAL_FRAME
        camera.transform.translation.z = CAMERA_HEIGHT_M
        camera.transform.rotation.x = 1.0
        camera.transform.rotation.w = 0.0
        self.static_tf.sendTransform(camera)

        # The probes are dynamic and bracketed -- one sample a second back, one a second ahead --
        # so a lookup at the image stamp and at the earlier stillness probe both interpolate, while
        # a stamp beyond the forward sample cannot be answered at all. Images are stamped when they
        # are built, which is always after this call, so the forward sample is what keeps them
        # inside the range.
        now = self.node.get_clock().now()
        samples = []
        for index in range(6):
            for stamp in (now - Duration(seconds=1.0), now + Duration(seconds=1.0)):
                probe = TransformStamped()
                probe.header.stamp = stamp.to_msg()
                probe.header.frame_id = "world"
                probe.child_frame_id = f"synthetic_probe_link_{index}"
                probe.transform.translation.x = 0.90
                probe.transform.translation.y = 0.90
                probe.transform.translation.z = 1.20
                probe.transform.rotation.w = 1.0
                samples.append(probe)
        self.dynamic_tf.sendTransform(samples)

    def _camera_info(self):
        info = CameraInfo()
        info.header.stamp = self.node.get_clock().now().to_msg()
        info.header.frame_id = OPTICAL_FRAME
        info.width = WIDTH
        info.height = HEIGHT
        info.distortion_model = "plumb_bob"
        info.k = [
            FOCAL,
            0.0,
            0.5 * WIDTH,
            0.0,
            FOCAL,
            0.5 * HEIGHT,
            0.0,
            0.0,
            1.0,
        ]
        return info

    def _depth_image(self):
        """Build a depth image whose only returns are one rectangular patch of surface."""
        nan = math.nan
        samples = array.array("f", [nan] * (WIDTH * HEIGHT))
        for row in PATCH_ROWS:
            for column in PATCH_COLUMNS:
                samples[row * WIDTH + column] = SURFACE_RANGE_M
        image = Image()
        image.header.stamp = self.node.get_clock().now().to_msg()
        image.header.frame_id = OPTICAL_FRAME
        image.width = WIDTH
        image.height = HEIGHT
        image.encoding = "32FC1"
        image.is_bigendian = 0
        image.step = WIDTH * struct.calcsize("f")
        image.data = array.array("B", samples.tobytes())
        return image

    def test_one_depth_image_becomes_one_placed_obstacle(self):
        """The patch must come back as a single box at the surface it was rendered on."""
        self._broadcast_frames()
        info = self._camera_info()

        def published():
            # Republished every attempt: unlike /tf_static, a dynamic transform only reaches
            # subscribers that were already listening when it was sent.
            self._broadcast_frames()
            self.info_publisher.publish(info)
            self.image_publisher.publish(self._depth_image())
            return self.observations[-1] if self.observations else None

        observation = self._spin_until(published, 60.0, "an obstacle observation")

        self.assertEqual(observation.header.frame_id, "world")
        self.assertEqual(observation.sensor_frame, OPTICAL_FRAME)
        self.assertGreaterEqual(observation.sequence, 1)
        # Nothing moved between the image stamp and the probe window; the projector only adopts
        # changed geometry from a still robot.
        self.assertTrue(observation.robot_static)
        self.assertEqual(len(observation.boxes), 1, "expected exactly one recovered obstacle")

        box = observation.boxes[0]
        expected_height = CAMERA_HEIGHT_M - SURFACE_RANGE_M
        self.assertAlmostEqual(box.center.x, 0.0, delta=0.05)
        self.assertAlmostEqual(box.center.y, 0.0, delta=0.05)
        self.assertAlmostEqual(box.center.z, expected_height, delta=0.05)
        expected_span = len(PATCH_COLUMNS) * SURFACE_RANGE_M / FOCAL
        self.assertAlmostEqual(box.size.x, expected_span, delta=0.05)
        self.assertAlmostEqual(box.size.y, expected_span, delta=0.05)
        self.assertGreater(box.point_count, 0)

    def test_z_a_frame_whose_transforms_are_missing_is_dropped_within_one_budget(self):
        """
        One transform budget per frame: a frame cannot queue its waits behind the next one.

        The self-filter probes are dynamic, so a frame stamped far ahead resolves the static sensor
        pose and then waits for each probe in turn. This test feeds those probes their transform
        one frame every 0.5 s, so the frame as a whole needs about 3.0 s of waiting to resolve --
        more than one `transform_timeout_sec` (2.0 s here), but each individual lookup would be
        served within one. The detector must refuse the frame at one budget instead: a frame that
        cannot resolve inside one `transform_timeout_sec` is dropped, and the frames it would hold
        back are published. Before Card 041 each lookup waited that timeout in turn, so the same
        frame was published at about 3.0 s, exactly the shape the loaded batch-2 receipt shows:
        frames holding the detector while the projector's 2.0 s obstacle window expires.
        """
        self._broadcast_frames()
        info = self._camera_info()

        def warm():
            self._broadcast_frames()
            self.info_publisher.publish(info)
            self.image_publisher.publish(self._depth_image())
            return self.observations or None

        self._spin_until(warm, 30.0, "the detector to publish its first observation")
        quiet = time.monotonic() + 0.5
        while time.monotonic() < quiet:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        last_sequence = self.observations[-1].sequence

        # Ahead of the warm-up samples so only the samples this test publishes on purpose can
        # answer these lookups -- but inside tf2's ten-second cache window, or the future samples
        # would evict the warm-up ones and poison every frame that follows.
        frame_stamp = self.node.get_clock().now() + Duration(seconds=8.0)
        bad = self._depth_image()
        bad.header.stamp = frame_stamp.to_msg()
        self.image_publisher.publish(bad)
        started = time.monotonic()

        def probe_sample(index, stamp):
            sample = TransformStamped()
            sample.header.stamp = stamp.to_msg()
            sample.header.frame_id = "world"
            sample.child_frame_id = f"synthetic_probe_link_{index}"
            sample.transform.translation.x = 0.90
            sample.transform.translation.y = 0.90
            sample.transform.translation.z = 1.20
            sample.transform.rotation.w = 1.0
            self.dynamic_tf.sendTransform(sample)

        due = {index: 0.5 * (index + 1) for index in range(6)}
        while time.monotonic() - started < 4.0:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            elapsed = time.monotonic() - started
            for index, at in list(due.items()):
                if elapsed >= at:
                    probe_sample(index, frame_stamp)
                    del due[index]
            leaked = [o for o in self.observations if o.sequence > last_sequence]
            self.assertFalse(
                leaked,
                f"the frame published at {time.monotonic() - started:.3f} s: it spent more than "
                "one transform budget waiting, holding back every frame behind it",
            )

        self._broadcast_frames()
        good_started = time.monotonic()

        def good():
            self._broadcast_frames()
            self.info_publisher.publish(self._camera_info())
            self.image_publisher.publish(self._depth_image())
            return [o for o in self.observations if o.sequence > last_sequence] or None

        fresh = self._spin_until(good, 30.0, "the frame after the unresolvable one")
        print(
            f"dropped the over-budget frame; successor arrived in "
            f"{time.monotonic() - good_started:.3f} s",
            flush=True,
        )
        self.assertEqual(
            len(fresh),
            1,
            "the over-budget frame published an obstacle without a complete self-filter volume",
        )
