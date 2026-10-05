# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Acceptance: the camera, not the simulator, is what the world state is told."""

import math
import statistics
import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import QoSProfile
from restocker_interfaces.msg import ObjectObservation, PerceptionFrame, PoseErrorSample
from restocker_interfaces.srv import GetWorldState
from sensor_msgs.msg import Image

ESTIMATE_TOPIC = "/perception/object_observations"
GROUND_TRUTH_TOPIC = "/perception/ground_truth/object_observations"
SAMPLE_TOPIC = "/perception/pose_error"
DETECTION_TOPIC = "/perception/detections"

PERCEPTION_BACKEND = "overhead_rgbd_colour_depth"
GROUND_TRUTH_BACKEND = "gazebo_ground_truth"

# The scenario stocks one of each. Identity comes from the inventory, so it appears on both topics;
# only the pose says which producer supplied it (test_c).
EXPECTED_CLASSES = {
    ObjectObservation.PRODUCT_CLASS_CAN,
    ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE,
    ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE,
}
GROUND_TRUTH_IDS = {
    "sim:stock_can_01",
    "sim:stock_small_bottle_01",
    "sim:stock_large_bottle_01",
}

# The world state's minimum_confidence in the launch composition.
CONFIGURED_MINIMUM_CONFIDENCE = 0.60

# Task selection rejects an observation older than 500 ms and the camera runs at 6 Hz, so a
# sustained cadence beyond this leaves no margin for the snapshot round trip. Applied to the
# median: one dropped render is the simulator's business.
MAXIMUM_OBSERVATION_GAP_S = 0.35

# A grasp of the narrowest product closes 66 mm of jaw around it. Measured error is a few
# millimetres; this bound is well above that so it fails on a defect, not noise.
MAXIMUM_TRANSLATION_ERROR_M = 0.02


def _seconds(stamp) -> float:
    """Convert a stamp to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


@pytest.mark.launch_test
def generate_test_description():
    """Start the simulation with the camera producing observations and ground truth measuring."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "gui": "false",
            "cameras": "true",
            "ground_truth": "true",
            # The producer swap: enable the pipeline and move ground truth off the observation
            # topic. Off by default because the swap deadlocks the retreat after a release, a
            # property of the executor's projection, not of what this test measures.
            "perception": "true",
            "object_observation_topic": "/perception/ground_truth/object_observations",
            "controller_timeout": "30.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([simulation, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestPerceptionRuntime(unittest.TestCase):
    """Assert the producer swap happened and measure it."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_runtime_test")
        cls.estimates = []
        cls.ground_truth = {}
        cls.samples = []
        cls.frames = []
        cls.acquisitions = []
        reliable = QoSProfile(depth=50)
        cls.node.create_subscription(
            ObjectObservation, ESTIMATE_TOPIC, cls.estimates.append, reliable
        )
        cls.node.create_subscription(
            ObjectObservation,
            GROUND_TRUTH_TOPIC,
            lambda message: cls.ground_truth.__setitem__(message.source_object_id, message),
            reliable,
        )
        cls.node.create_subscription(PoseErrorSample, SAMPLE_TOPIC, cls.samples.append, reliable)
        cls.node.create_subscription(
            PerceptionFrame, DETECTION_TOPIC, cls.frames.append, QoSProfile(depth=5)
        )
        # Reliable, not qos_profile_sensor_data: these frames are megabytes each and best-effort
        # delivers only a fraction of them (measured at 11% of /overhead_camera/image in the same
        # full-suite window that measured the stream at 6.024 Hz reliably in test_camera_runtime).
        # The stamps are simulation time, so what this recorder must see is the sensor's
        # acquisitions, not the transport's survivors — the same reason perception itself
        # subscribes reliably (perception.reliable_sensor_qos).
        cls.node.create_subscription(
            Image,
            "/overhead_camera/image",
            lambda message: cls.acquisitions.append(_seconds(message.header.stamp)),
            QoSProfile(depth=50),
        )
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_sec, failure):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            value = predicate()
            if value:
                return value
        self.fail(failure)

    def _spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _tracked_classes(self):
        return {observation.product_class for observation in self.estimates}

    def test_a_the_camera_is_what_publishes_object_observations(self):
        """Every observation on the executor's topic comes from the camera backend."""
        self._spin_until(
            lambda: self._tracked_classes() >= EXPECTED_CLASSES,
            240.0,
            "the perception pipeline never reported all three stocked categories on "
            f"{ESTIMATE_TOPIC}; saw {self._tracked_classes()}",
        )
        backends = {observation.backend_name for observation in self.estimates}
        self.assertEqual(
            backends,
            {PERCEPTION_BACKEND},
            f"{ESTIMATE_TOPIC} must carry only camera-derived observations, saw {backends}",
        )
        self.assertNotIn(GROUND_TRUTH_BACKEND, backends)
        # Lane policy admits by class and SKU; a missing SKU would only surface later, at the
        # destination lane, as an incompatible identity.
        for observation in self.estimates:
            self.assertTrue(
                observation.has_sku and observation.sku,
                f"an observation of class {observation.product_class} named no SKU",
            )
        # The detection boundary is published, stamped and framed by its own acquisition.
        self.assertTrue(self.frames, "the detection boundary was never published")
        for frame in self.frames:
            self.assertEqual(frame.status, PerceptionFrame.STATUS_OK, frame.status_detail)
            self.assertEqual(frame.header.frame_id, "overhead_camera_optical_frame")
            self.assertEqual(frame.backend_name, PERCEPTION_BACKEND)

    def test_b_ground_truth_still_publishes_but_on_its_own_topic(self):
        """Ground truth keeps its measurement role and loses its authoritative one."""
        self._spin_until(
            lambda: set(self.ground_truth) >= GROUND_TRUTH_IDS,
            180.0,
            f"simulator ground truth never reported every stocked product on {GROUND_TRUTH_TOPIC}",
        )
        for observation in self.ground_truth.values():
            self.assertEqual(observation.backend_name, GROUND_TRUTH_BACKEND)

    def test_c_the_world_state_tracks_what_the_camera_saw_and_not_what_gazebo_knows(self):
        """The authoritative pose of every product is the camera's, not the simulator's."""
        self.assertTrue(
            self.world_state.wait_for_service(timeout_sec=60.0),
            "the world state never offered its snapshot service",
        )

        def snapshot():
            future = self.world_state.call_async(GetWorldState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
            result = future.result()
            if result is None or len(result.snapshot.objects) < len(EXPECTED_CLASSES):
                return None
            return result.snapshot

        tracked = self._spin_until(
            snapshot, 180.0, "the world state never admitted all three products"
        )
        self._spin_until(
            lambda: set(self.ground_truth) >= GROUND_TRUTH_IDS,
            120.0,
            "ground truth to arrive for the comparison below",
        )
        self.assertEqual(
            {obj.product_class for obj in tracked.objects},
            EXPECTED_CLASSES,
            "the world state must track exactly the categories the scenario stocks",
        )
        # Every tracked pose must be the camera's estimate: a few millimetres from ground truth
        # and never exactly equal. An identical pose would mean the simulator's value came through.
        for tracked_object in tracked.objects:
            self.assertTrue(
                tracked_object.has_sku and tracked_object.sku,
                f"world state tracks {tracked_object.source_object_id} with no SKU, so no lane "
                "will accept it",
            )
            truth = self.ground_truth.get(tracked_object.source_object_id)
            self.assertIsNotNone(
                truth, f"no ground truth for {tracked_object.source_object_id} to compare against"
            )
            offset = math.dist(
                (
                    tracked_object.pose.pose.position.x,
                    tracked_object.pose.pose.position.y,
                    tracked_object.pose.pose.position.z,
                ),
                (
                    truth.pose.pose.position.x,
                    truth.pose.pose.position.y,
                    truth.pose.pose.position.z,
                ),
            )
            print(
                f"{tracked_object.source_object_id}: world state is "
                f"{offset * 1000:.2f} mm from simulator ground truth"
            )
            self.assertGreater(
                offset,
                0.0,
                f"{tracked_object.source_object_id} is tracked at exactly its ground-truth pose, "
                "so the world state is still being fed by the simulator",
            )
            self.assertLess(offset, MAXIMUM_TRANSLATION_ERROR_M)

    def test_d_observations_clear_the_confidence_and_freshness_the_executor_needs(self):
        """Observations are confident and fresh enough for the executor."""
        self._spin_until(
            lambda: self._tracked_classes() >= EXPECTED_CLASSES, 240.0, "no observations arrived"
        )
        self.estimates.clear()
        self.acquisitions.clear()
        self._spin_for(30.0)
        # The window's frames are snapshotted before a short drain: reliable delivery makes every
        # acquisition visible, including the last few whose observations are still in flight, and
        # the completeness check below must compare against that window rather than racing it.
        window = sorted(set(self.acquisitions))
        self._spin_for(1.0)
        self.assertTrue(self.estimates, "the pipeline stopped publishing")

        confidences = [observation.confidence for observation in self.estimates]
        print(
            f"confidence over {len(confidences)} observations: "
            f"min {min(confidences):.3f} mean {statistics.fmean(confidences):.3f} "
            f"max {max(confidences):.3f}"
        )
        self.assertGreaterEqual(
            min(confidences),
            CONFIGURED_MINIMUM_CONFIDENCE,
            "an observation below the configured threshold reached the topic, which means the "
            "world state is discarding it and the object is silently ageing out",
        )

        by_identity = {}
        for observation in self.estimates:
            by_identity.setdefault(observation.source_object_id, []).append(
                _seconds(observation.header.stamp)
            )
        self.assertEqual(
            len(by_identity),
            len(EXPECTED_CLASSES),
            f"identity was not stable across the window: {sorted(by_identity)}",
        )

        # Asserts that perception reports every acquisition it is given, not the camera's rate
        # (test_camera_runtime covers that; a loaded machine drops frames). Observations carry the
        # acquisition stamp, so the two stamp sets are directly comparable.
        acquisitions = window
        self.assertGreater(len(acquisitions), 5, "the camera produced almost no acquisitions")
        interior = set(acquisitions[1:-1])
        for identity, stamps in sorted(by_identity.items()):
            stamps.sort()
            gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:], strict=False)]
            missed = interior - set(stamps)
            print(
                f"{identity}: {len(stamps)} observations over {len(acquisitions)} "
                f"acquisitions, worst gap {max(gaps) * 1000:.0f} ms, median "
                f"{statistics.median(gaps) * 1000:.0f} ms, unreported acquisitions: {len(missed)}"
            )
            self.assertLessEqual(
                len(missed),
                1,
                f"perception saw {len(missed)} acquisitions without reporting {identity}, so the "
                "gaps are its own rather than the sensor's",
            )
        camera_gaps = [
            later - earlier for earlier, later in zip(acquisitions, acquisitions[1:], strict=False)
        ]
        print(
            f"overhead acquisitions: worst gap {max(camera_gaps) * 1000:.0f} ms, "
            f"median {statistics.median(camera_gaps) * 1000:.0f} ms"
        )
        # Task selection calls an object stale after 500 ms; the median cadence must leave room.
        self.assertLess(
            statistics.median(camera_gaps),
            MAXIMUM_OBSERVATION_GAP_S,
            "the sustained observation cadence is too slow for the executor's freshness gate",
        )

    def test_e_pose_error_is_measured_against_simulator_ground_truth(self):
        """Pose error against simulator ground truth, reported with its numbers."""
        self._spin_until(
            lambda: (
                {
                    sample.source_object_id
                    for sample in self.samples
                    if sample.status == PoseErrorSample.STATUS_OK
                }
                >= GROUND_TRUTH_IDS
            ),
            240.0,
            "the evaluator never produced an OK sample for every stocked product",
        )
        measured = {}
        for sample in self.samples:
            if sample.status != PoseErrorSample.STATUS_OK:
                continue
            self.assertEqual(sample.header.frame_id, "world")
            self.assertEqual(sample.backend_name, PERCEPTION_BACKEND)
            # The evaluator refuses to pair beyond its stamp tolerance.
            skew = abs(
                (sample.estimate_stamp.sec + sample.estimate_stamp.nanosec * 1e-9)
                - (sample.ground_truth_stamp.sec + sample.ground_truth_stamp.nanosec * 1e-9)
            )
            self.assertLess(skew, 0.2001, "a sample was paired beyond the stamp tolerance")
            # Read the orientation status, not the angle: this backend publishes a constant
            # orientation, so samples report ORIENTATION_NOT_ESTIMATED with a NaN, and a printed
            # 0.0000 rad would make an orientation-blind pipeline look perfect.
            self.assertIn(
                sample.orientation_status,
                (
                    PoseErrorSample.ORIENTATION_NOT_ESTIMATED,
                    PoseErrorSample.ORIENTATION_AXIS_ESTIMATED,
                ),
            )
            if sample.orientation_status == PoseErrorSample.ORIENTATION_NOT_ESTIMATED:
                self.assertTrue(
                    math.isnan(sample.axis_error_rad),
                    "a sample that estimated no orientation still carried a readable angle",
                )
            measured.setdefault(sample.source_object_id, []).append(
                (sample.translation_error_m, sample.orientation_status, sample.axis_error_rad)
            )

        print("pose error against simulator ground truth:")
        for source_object_id, errors in sorted(measured.items()):
            translations = [error[0] for error in errors]
            estimated = PoseErrorSample.ORIENTATION_AXIS_ESTIMATED
            axes = [error[2] for error in errors if error[1] == estimated]
            orientation = (
                f"worst axis error {max(axes):.4f} rad over {len(axes)} samples"
                if axes
                else "orientation not estimated by this backend"
            )
            print(
                f"  {source_object_id}: {len(errors)} samples, "
                f"mean {statistics.fmean(translations) * 1000:.2f} mm, "
                f"worst {max(translations) * 1000:.2f} mm; {orientation}"
            )
            self.assertLess(
                max(translations),
                MAXIMUM_TRANSLATION_ERROR_M,
                f"{source_object_id} was estimated {max(translations) * 1000:.1f} mm from where "
                "the simulator says it is",
            )
        self.assertEqual(set(measured), GROUND_TRUTH_IDS)
