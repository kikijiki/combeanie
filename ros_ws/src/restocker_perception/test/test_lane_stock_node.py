# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/perception/lane_stock over a real ROS graph: a synthetic survey frame, fake services."""
# One process on a private DDS domain. A fake survey publishes what the wrist survey does (depth,
# RGB, camera_info, the lane <- optical transform, a LaneObservation) and the node must answer with
# a LaneStock: counted from the observation's free depth and the frame, policy from the (fake)
# world state, identity from the (fake or absent) sidecar. The estimator itself is covered without
# ROS in test_lane_stock.py and test_lane_stock_pipeline.py.

import math
import os
from pathlib import Path
import random
import sys
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from builtin_interfaces.msg import Time  # noqa: E402,I100,I101
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from restocker_interfaces.msg import LaneObservation, LaneStock, ShelfLane  # noqa: E402
from restocker_interfaces.srv import GetWorldState, IdentifyProduct  # noqa: E402
from restocker_perception.stock.node import LaneStockNode, depth_to_metres  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from test_lane_stock_pipeline import CONFIG, HEIGHT, WIDTH, render, station_pose  # noqa: E402,I100
from tf2_ros import StaticTransformBroadcaster  # noqa: E402


@pytest.fixture(scope="module")
def ros_context():
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=random.Random(os.getpid() + 7).randint(120, 200))
    yield context
    rclpy.shutdown(context=context)


def quaternion(matrix):
    w = math.sqrt(max(0.0, 1.0 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2])) / 2.0
    return (
        (matrix[2, 1] - matrix[1, 2]) / (4 * w),
        (matrix[0, 2] - matrix[2, 0]) / (4 * w),
        (matrix[1, 0] - matrix[0, 1]) / (4 * w),
        w,
    )


class Survey:
    """A fake wrist survey plus the world state, publishing one frame and one observation."""

    def __init__(self, context, depth, expected_class, expected_sku, identify=None):
        self.node = rclpy.create_node("fake_survey", context=context)
        self.stamp = Time(sec=12, nanosec=500_000_000)
        self.depth_pub = self.node.create_publisher(Image, "/wrist_camera/depth_image", 10)
        self.image_pub = self.node.create_publisher(Image, "/wrist_camera/image", 10)
        self.info_pub = self.node.create_publisher(CameraInfo, "/wrist_camera/camera_info", 10)
        self.obs_pub = self.node.create_publisher(
            LaneObservation, "/perception/lane_observations", 10
        )
        self.tf = StaticTransformBroadcaster(self.node)
        pose = station_pose()
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "lane_01", "wrist_optical"
        transform.header.stamp = self.stamp
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = pose[:3, 3]
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion(pose[:3, :3])
        self.tf.sendTransform(transform)

        def world(_request, response):
            lane = ShelfLane()
            lane.id, lane.expected_product_class = "lane_01", expected_class
            lane.has_expected_sku, lane.expected_sku = True, expected_sku
            response.snapshot.lanes = [lane]
            return response

        self.world = self.node.create_service(GetWorldState, "/world_state/get_snapshot", world)
        self.identify_service = None
        if identify is not None:
            self.identify_service = self.node.create_service(
                IdentifyProduct, "/identify_product", identify
            )
        self.depth = depth

    def publish(self, free_depth, coverage=1.0, status=LaneObservation.STATUS_OK):
        header_time = self.stamp
        depth = Image()
        depth.header.stamp, depth.header.frame_id = header_time, "wrist_optical"
        depth.height, depth.width, depth.encoding, depth.step = HEIGHT, WIDTH, "32FC1", WIDTH * 4
        depth.data = self.depth.astype(np.float32).tobytes()
        image = Image()
        image.header = depth.header
        image.height, image.width, image.encoding, image.step = HEIGHT, WIDTH, "rgb8", WIDTH * 3
        image.data = np.full((HEIGHT, WIDTH, 3), 120, dtype=np.uint8).tobytes()
        info = CameraInfo()
        info.header = depth.header
        info.width, info.height = WIDTH, HEIGHT
        info.k = [700.9, 0.0, 640.0, 0.0, 700.9, 360.0, 0.0, 0.0, 1.0]
        observation = LaneObservation()
        observation.header.stamp, observation.header.frame_id = header_time, "lane_01"
        observation.lane_id, observation.available_depth_m = "lane_01", free_depth
        observation.confidence, observation.status = coverage, status
        for _ in range(3):  # reliable enough for a one-process graph
            self.info_pub.publish(info)
            self.depth_pub.publish(depth)
            self.image_pub.publish(image)
            time.sleep(0.05)
        self.obs_pub.publish(observation)


def run(context, survey, expected_wait=15.0):
    node = LaneStockNode(context=context)
    received = []
    listener = rclpy.create_node("listener", context=context)
    listener.create_subscription(LaneStock, "/perception/lane_stock", received.append, 10)
    executor = MultiThreadedExecutor(context=context, num_threads=4)
    for member in (node, survey.node, listener):
        executor.add_node(member)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    return node, listener, executor, received


def finish(node, listener, survey, executor):
    executor.shutdown()
    node.pending.put(None)
    for member in (node, listener, survey.node):
        member.destroy_node()


def wait_for(received, seconds=15.0):
    deadline = time.time() + seconds
    while time.time() < deadline and not received:
        time.sleep(0.05)
    return received[0] if received else None


def test_depth_image_conversion():
    message = Image(height=2, width=3, encoding="16UC1", step=8)
    message.data = np.array([[1000, 2000, 3000, 0], [500, 0, 0, 0]], dtype=np.uint16).tobytes()
    assert depth_to_metres(message)[0, 1] == pytest.approx(2.0)


def test_counts_a_column_with_the_sidecar_absent(ros_context):
    geometry = CONFIG.products["can"]
    depth = render([(geometry, 4)], station_pose())
    survey = Survey(ros_context, depth, geometry.product_class, "GEN-001")
    node, listener, executor, received = run(ros_context, survey)
    survey.publish(CONFIG.free_depth_of_column(geometry, 4))
    stock = wait_for(received)
    finish(node, listener, survey, executor)
    assert stock is not None, "no LaneStock was published"
    assert stock.state == LaneStock.STATE_ESTIMATED
    assert stock.count_estimate == 4 and stock.count_min == stock.count_max == 4
    assert stock.lane_id == "lane_01" and stock.expected_sku == "GEN-001"
    assert not stock.identity_channel_reachable
    assert stock.composition == LaneStock.COMPOSITION_UNKNOWN and not stock.misplaced


def test_low_coverage_observation_is_a_cannot_tell(ros_context):
    geometry = CONFIG.products["can"]
    depth = render([(geometry, 0)], station_pose())
    survey = Survey(ros_context, depth, geometry.product_class, "GEN-001")
    node, listener, executor, received = run(ros_context, survey)
    survey.publish(CONFIG.workcell.usable_depth_m, coverage=0.2)
    stock = wait_for(received)
    finish(node, listener, survey, executor)
    assert stock is not None
    assert (
        stock.state == LaneStock.STATE_CANNOT_TELL
        and stock.reason == LaneStock.REASON_LOW_COVERAGE
    )
    assert stock.count_estimate == 0


def test_a_confirmed_wrong_name_from_the_sidecar_flags_the_lane(ros_context):
    geometry = CONFIG.products["large_bottle"]
    depth = render([(geometry, 2)], station_pose())

    def identify(_request, response):
        response.status, response.sidecar_available = (
            IdentifyProduct.Response.STATUS_IDENTIFIED,
            True,
        )
        response.has_sku, response.sku = True, "GEN-002"
        response.evidence_kinds, response.evidence_product_ids = (
            ["embedding", "barcode"],
            ["GEN-002", "GEN-002"],
        )
        response.registry_sha256 = "reg"
        return response

    survey = Survey(ros_context, depth, geometry.product_class, "GEN-001", identify=identify)
    node, listener, executor, received = run(ros_context, survey)
    survey.publish(CONFIG.free_depth_of_column(geometry, 2))
    stock = wait_for(received, 25.0)
    finish(node, listener, survey, executor)
    assert stock is not None
    assert (
        stock.count_estimate == 2 and stock.misplaced and list(stock.misplaced_skus) == ["GEN-002"]
    )
    assert stock.composition == LaneStock.COMPOSITION_MISPLACED
    assert stock.identity_channel_reachable and stock.identity_registry_sha256 == "reg"


def test_observation_without_a_matching_frame_is_counted_from_its_free_depth(ros_context):
    geometry = CONFIG.products["can"]
    depth = render([(geometry, 3)], station_pose())
    survey = Survey(ros_context, depth, geometry.product_class, "GEN-001")
    node, listener, executor, received = run(ros_context, survey)
    observation = LaneObservation()
    observation.header.stamp = Time(sec=99, nanosec=0)  # no frame has this stamp
    observation.header.frame_id, observation.lane_id = "lane_01", "lane_01"
    observation.available_depth_m = CONFIG.free_depth_of_column(geometry, 3)
    observation.confidence, observation.status = 1.0, LaneObservation.STATUS_OK
    survey.obs_pub.publish(observation)
    stock = wait_for(received)
    finish(node, listener, survey, executor)
    assert stock is not None
    assert stock.state == LaneStock.STATE_ESTIMATED and stock.count_estimate == 3
    assert stock.visible_faces == 0 and not stock.identity_channel_reachable


def test_a_refused_observation_is_a_refusal(ros_context):
    geometry = CONFIG.products["can"]
    survey = Survey(ros_context, render([], station_pose()), geometry.product_class, "GEN-001")
    node, listener, executor, received = run(ros_context, survey)
    survey.publish(0.0, coverage=1.0, status=LaneObservation.STATUS_INSUFFICIENT_COVERAGE)
    stock = wait_for(received)
    finish(node, listener, survey, executor)
    assert stock is not None
    assert (
        stock.state == LaneStock.STATE_CANNOT_TELL and stock.reason == LaneStock.REASON_NO_EVIDENCE
    )
