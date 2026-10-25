# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Perception benchmark recorder: wrist and overhead frames of one scenario with ground truth."""

# Card 109. Launches the public simulation with both cameras and ground truth, waits until every
# product the scenario spawns is where the scenario says (a label that disagrees with the world
# would poison the benchmark, so that is a failure, not a warning), then records FRAMES frames per
# camera. Each frame is written as PNG (RGB, plus depth in millimetres as 16-bit PNG) beside a
# label row built by restocker_benchmarks.perception_dataset.frame_label from the scenario, the
# camera pose from TF at the image stamp, and the camera_info intrinsics.
#
# With nothing set it records the shipped sensor-acceptance scenario into a temporary directory
# (the one simulator run of the unit suite). tools/record_perception_dataset.py sets:
#   RESTOCKER_BENCH_SCENARIO     scenario YAML to launch
#   RESTOCKER_BENCH_TRUTH        truth JSON written by `benchmark_perception scenes`
#   RESTOCKER_BENCH_SCENARIO_ID  stable scene id, e.g. seed000007
#   RESTOCKER_BENCH_SEED / RESTOCKER_BENCH_SPLIT   seed and its frozen split
#   RESTOCKER_BENCH_PART_DIR     where the part (frames/, rows.jsonl, part.json) is written
#   RESTOCKER_BENCH_FRAMES       frames per camera (default 2)
#   RESTOCKER_BENCH_WORLD        world SDF to launch instead of the shipped one (seeded lighting)
#   RESTOCKER_BENCH_LIGHTING_JSON  the lighting object written into every label row
#   RESTOCKER_BENCH_VIEWS        JSON list of close-range wrist viewpoints (perception_views):
#                                the arm is sent to each through /survey_viewpoint, so the planner
#                                and controllers move it, and one more wrist frame is recorded
#                                from where it arrived. The composition then includes move_group.
#                                A view with "range_check": false (a survey station of
#                                perception_proposals) has no target product to range.

import json
import math
import os
from pathlib import Path
import tempfile
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
from rclpy.action import ActionClient
from rclpy.time import Time
from restocker_benchmarks import perception_dataset as dataset
from restocker_interfaces.action import SurveyViewpoint
from restocker_interfaces.msg import ObjectObservation
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import yaml

SCENARIO = os.environ.get("RESTOCKER_BENCH_SCENARIO", "")
TRUTH = os.environ.get("RESTOCKER_BENCH_TRUTH", "")
SCENARIO_ID = os.environ.get("RESTOCKER_BENCH_SCENARIO_ID", "shipped-sensor-acceptance")
SEED = os.environ.get("RESTOCKER_BENCH_SEED", "")
SPLIT = os.environ.get("RESTOCKER_BENCH_SPLIT", "") or None
PART_DIR = os.environ.get("RESTOCKER_BENCH_PART_DIR", "")
FRAMES = int(os.environ.get("RESTOCKER_BENCH_FRAMES", "2") or "2")
WORLD = os.environ.get("RESTOCKER_BENCH_WORLD", "")
VIEWS = os.environ.get("RESTOCKER_BENCH_VIEWS", "")

CAMERAS = {
    "overhead": ("/overhead_camera", "overhead_camera_optical_frame"),
    "wrist": ("/wrist_camera", "wrist_camera_optical_frame"),
}
# With viewpoints the survey server's motion port needs the world state fed from ground truth, so
# ground truth stays on the topic the world state ingests (as test_wrist_viewpoint_runtime does).
GROUND_TRUTH_TOPIC = (
    "/perception/object_observations" if VIEWS else "/perception/ground_truth/object_observations"
)
# A close frame must have its target product this near the lens to count as readable.
CLOSE_RANGE_M = (0.2, 0.55)
VIEW_TIMEOUT_S = 240.0
# How far a spawned product may sit from its scenario pose before the label is untrustworthy.
SETTLE_TOLERANCE_M = 0.03
# Minimum simulation time between two recorded frames of one camera.
FRAME_SPACING_S = 1.0


@pytest.mark.launch_test
def generate_test_description():
    """Start the public simulation composition with cameras and ground truth enabled."""
    arguments = {
        "gui": "false",
        "cameras": "true",
        "wrist_camera": "true",
        "ground_truth": "true",
        "perception": "false",
        "object_observation_topic": GROUND_TRUTH_TOPIC,
        "controller_timeout": "30.0",
    }
    arguments["scenario_config"] = SCENARIO or str(
        Path(get_package_share_directory("restocker_gazebo"))
        / "config"
        / "sensor_acceptance_products.yaml"
    )
    if WORLD:
        arguments["world"] = WORLD
    launch_file = "simulation.launch.py"
    if VIEWS:
        launch_file = "baseline.launch.py"
        arguments.update(
            {
                "lane_observation_topic": "/perception/ground_truth/lane_observations",
                "tray_overview_perception": "false",
                "tray_confirm_perception": "false",
                "rviz": "false",
                "task_coordinator": "false",
                "attachment_adapter": "false",
                "planning_smoke": "false",
                "planning_scene_projection": "true",
                "survey_viewpoint": "true",
                # The depth-obstacle pipeline is not part of what is recorded; left on, the
                # planning scene waits for obstacle evidence the benchmark never produces.
                "obstacle_perception": "false",
                "controller_timeout": "60.0",
            }
        )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("restocker_bringup"), "launch", launch_file])
        ),
        launch_arguments=arguments.items(),
    )
    return (
        launch.LaunchDescription([simulation, launch_testing.actions.ReadyToTest()]),
        {},
    )


def _image_to_array(message: Image, wanted: str) -> numpy.ndarray:
    """Convert an Image message to an RGB uint8 or a 16-bit millimetre depth array."""
    height, width = message.height, message.width
    raw = numpy.frombuffer(bytes(message.data), dtype=numpy.uint8)
    if wanted == "rgb":
        if message.encoding not in {"rgb8", "bgr8"}:
            raise AssertionError(f"unexpected colour encoding {message.encoding}")
        array = raw.reshape(height, message.step)[:, : width * 3].reshape(height, width, 3)
        return array[:, :, ::-1].copy() if message.encoding == "bgr8" else array.copy()
    if message.encoding == "32FC1":
        floats = numpy.frombuffer(bytes(message.data), dtype=numpy.float32).reshape(height, width)
        millimetres = numpy.where(numpy.isfinite(floats), floats * 1000.0, 0.0)
        return numpy.clip(millimetres, 0, 65535).astype(numpy.uint16)
    if message.encoding == "16UC1":
        return numpy.frombuffer(bytes(message.data), dtype=numpy.uint16).reshape(height, width)
    raise AssertionError(f"unexpected depth encoding {message.encoding}")


def _stamp_s(message) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class TestPerceptionRecorder(unittest.TestCase):
    """Record labelled frames once the world matches the scenario."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_recorder")
        reliable = rclpy.qos.QoSProfile(depth=20, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        cls.latest = {}
        cls.subscriptions = []
        for _sensor, (prefix, _frame) in CAMERAS.items():
            for suffix, message_type in (
                ("image", Image),
                ("depth_image", Image),
                ("camera_info", CameraInfo),
            ):
                topic = f"{prefix}/{suffix}"
                cls.subscriptions.append(
                    cls.node.create_subscription(
                        message_type, topic, cls._make_store(topic), reliable
                    )
                )
        cls.observations = {}
        cls.subscriptions.append(
            cls.node.create_subscription(
                ObjectObservation,
                GROUND_TRUTH_TOPIC,
                cls._on_observation,
                rclpy.qos.QoSProfile(depth=50),
            )
        )
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.survey = ActionClient(cls.node, SurveyViewpoint, "/survey_viewpoint")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _make_store(cls, topic):
        def store(message):
            cls.latest[topic] = message

        return store

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

    def _grab_frame(self, prefix, optical_frame, not_before):
        """Return (image, depth, info, transform) from one render pass, or None until available."""
        # The transform is taken at the image's own stamp, never "latest", so the pose is the one
        # the image was rendered from. The wrist chain only exists once the controllers are up,
        # so a failed lookup is retried against the next image rather than the same stale stamp.
        topics = [f"{prefix}/{s}" for s in ("image", "depth_image", "camera_info")]
        if not all(topic in self.latest for topic in topics):
            return None
        image, depth, info = (self.latest[topic] for topic in topics)
        if _stamp_s(image) < not_before + FRAME_SPACING_S:
            return None
        if image.header.stamp != depth.header.stamp:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", optical_frame, Time.from_msg(image.header.stamp)
            )
        except (
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.LookupException,
        ):
            return None
        return image, depth, info, transform

    def _survey_to(self, view: dict) -> dict:
        """Send the arm to one viewpoint through the survey action; report how it ended."""
        goal = SurveyViewpoint.Goal()
        goal.label = view["name"]
        goal.camera_optical_pose.header.frame_id = "world"
        pose = goal.camera_optical_pose.pose
        pose.position.x, pose.position.y, pose.position.z = view["camera_optical_pose"][:3]
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = view["camera_optical_pose"][3:]
        sent = self.survey.send_goal_async(goal)
        self._spin_until(sent.done, 30.0, "survey goal was not answered")
        handle = sent.result()
        if not handle.accepted:
            return {"arrived": False, "detail": "goal rejected"}
        done = handle.get_result_async()
        self._spin_until(done.done, VIEW_TIMEOUT_S, f"survey {view['name']} never finished")
        result = done.result().result
        return {
            "arrived": result.outcome == SurveyViewpoint.Result.OUTCOME_ARRIVED,
            "outcome": int(result.outcome),
            "detail": result.detail,
            "achieved_translation_error_m": result.achieved_translation_error_m,
            "achieved_rotation_error_rad": result.achieved_rotation_error_rad,
        }

    def _scenario_document(self) -> dict:
        path = (
            Path(SCENARIO)
            if SCENARIO
            else Path(get_package_share_directory("restocker_gazebo"))
            / "config"
            / "sensor_acceptance_products.yaml"
        )
        return yaml.safe_load(path.read_text(encoding="utf-8"))

    def _truth(self, scenario: dict) -> dict:
        if TRUTH:
            return json.loads(Path(TRUTH).read_text(encoding="utf-8"))
        return dataset.scene_truth(scenario)

    def test_record_labelled_frames(self):
        scenario = self._scenario_document()
        truth = self._truth(scenario)

        # 1. The world must match the scenario before anything is recorded.
        expected = {p["source_object_id"]: p["spawn_pose"][:3] for p in scenario["products"]}
        self._spin_until(
            lambda: set(expected) <= set(self.observations),
            240.0,
            f"ground truth never reported {sorted(set(expected) - set(self.observations))}",
        )

        def position(name: str) -> list[float]:
            point = self.observations[name].pose.pose.position
            return [point.x, point.y, point.z]

        def settled() -> bool:
            return all(
                math.dist(position(name), pose) <= SETTLE_TOLERANCE_M
                for name, pose in expected.items()
            )

        self._spin_until(settled, 60.0, "products did not settle at their scenario poses")
        worst = max(math.dist(position(n), p) for n, p in expected.items())

        # 2. Record frames.
        part = Path(PART_DIR) if PART_DIR else Path(tempfile.mkdtemp(prefix="perception-part-"))
        (part / "frames").mkdir(parents=True, exist_ok=True)
        lighting = dataset.world_lighting(
            Path(get_package_share_directory("restocker_gazebo")) / "worlds" / "restocking.sdf"
        )
        lighting_override = os.environ.get("RESTOCKER_BENCH_LIGHTING_JSON", "")
        if lighting_override:
            lighting = json.loads(Path(lighting_override).read_text(encoding="utf-8"))
        rows = []

        def record(sensor, index, grabbed, last_stamp, viewpoint=None):
            image, depth, info, transform = grabbed
            t, q = transform.transform.translation, transform.transform.rotation
            frame_id = f"{SCENARIO_ID}-{sensor}-{index:03d}"
            (part / "frames" / f"{frame_id}.png").write_bytes(
                dataset.encode_png(_image_to_array(image, "rgb"))
            )
            (part / "frames" / f"{frame_id}.depth.png").write_bytes(
                dataset.encode_png(_image_to_array(depth, "depth"))
            )
            row = dataset.frame_label(
                frame_id=frame_id,
                scenario=SCENARIO_ID,
                seed=int(SEED) if SEED else None,
                seed_split=SPLIT,
                sensor=sensor,
                index=index,
                truth=truth,
                camera_pose_world=[t.x, t.y, t.z, q.x, q.y, q.z, q.w],
                intrinsics={
                    "width": info.width,
                    "height": info.height,
                    "fx": info.k[0],
                    "fy": info.k[4],
                    "cx": info.k[2],
                    "cy": info.k[5],
                },
                lighting=lighting,
                files={
                    "rgb": f"frames/{frame_id}.png",
                    "depth_mm": f"frames/{frame_id}.depth.png",
                },
                stamp_sim_s=last_stamp,
                viewpoint=viewpoint,
            )
            rows.append(row)
            return row

        def next_frame(sensor, prefix, optical_frame, not_before):
            grabbed = []

            def ready():
                result = self._grab_frame(prefix, optical_frame, not_before)
                grabbed[:] = [] if result is None else [result]
                return result is not None

            self._spin_until(ready, 180.0, f"{sensor} camera never delivered a labelled frame")
            return grabbed[0]

        for sensor, (prefix, optical_frame) in CAMERAS.items():
            last_stamp = -1e9
            for index in range(FRAMES):
                grabbed = next_frame(sensor, prefix, optical_frame, last_stamp)
                last_stamp = _stamp_s(grabbed[0])
                record(sensor, index, grabbed, last_stamp)

        # 3. Close-range wrist viewpoints: the planner and controllers move the arm there.
        views = json.loads(Path(VIEWS).read_text(encoding="utf-8")) if VIEWS else []
        view_results = []
        prefix, optical_frame = CAMERAS["wrist"]
        self.assertTrue(
            not views or self.survey.wait_for_server(timeout_sec=120.0),
            "/survey_viewpoint is not available",
        )
        for number, view in enumerate(views):
            outcome = self._survey_to(view)
            result = {
                "name": view["name"],
                "target_object_id": view["target_object_id"],
                **outcome,
            }
            if outcome["arrived"]:
                latest = self.latest.get(f"{prefix}/image")
                settle_from = _stamp_s(latest) if latest is not None else 0.0
                grabbed = next_frame("wrist", prefix, optical_frame, settle_from)
                row = record(
                    "wrist", FRAMES + number, grabbed, _stamp_s(grabbed[0]), viewpoint=view["name"]
                )
                result.update(frame_id=row["frame_id"], range_check=view.get("range_check", True))
                if result["range_check"]:
                    target = next(
                        o for o in row["objects"] if o["object_id"] == view["target_object_id"]
                    )
                    result.update(
                        target_in_view=target["in_view"], target_range_m=target["range_m"]
                    )
            view_results.append(result)
            print(f"perception recorder: view {view['name']}: {result}")
        (part / "rows.jsonl").write_text(
            "".join(dataset.canonical_json(row) + "\n" for row in rows), encoding="utf-8"
        )
        (part / "part.json").write_text(
            json.dumps(
                {
                    "scenario_id": SCENARIO_ID,
                    "seed": int(SEED) if SEED else None,
                    "seed_split": SPLIT,
                    "frames_per_camera": FRAMES,
                    "max_settle_error_m": worst,
                    "views": view_results,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"perception recorder: {len(rows)} frames in {part}, settle error {worst:.4f} m")

        # 4. The record itself is checked: every row has both files and non-empty labels.
        arrived = [r for r in view_results if r["arrived"]]
        self.assertEqual(len(rows), FRAMES * len(CAMERAS) + len(arrived))
        if views:
            self.assertTrue(arrived, f"no viewpoint was reached: {view_results}")
        for result in arrived:
            if not result["range_check"]:
                continue  # a survey-station view (CMB-113) has no target product to range
            low, high = CLOSE_RANGE_M
            self.assertTrue(
                result["target_in_view"] and low <= result["target_range_m"] <= high,
                f"close view {result['name']} does not hold its target: {result}",
            )
        for row in rows:
            self.assertTrue(row["objects"], "a frame without labelled objects")
            for relative in row["files"].values():
                self.assertGreater((part / relative).stat().st_size, 100)
            intr = row["camera"]["intrinsics"]
            self.assertGreater(intr["fx"], 0.0)
        overhead = [r for r in rows if r["sensor"] == "overhead"]
        self.assertTrue(
            any(o["in_view"] for o in overhead[0]["objects"]),
            "no labelled object projects into the overhead image: the label geometry is wrong",
        )
