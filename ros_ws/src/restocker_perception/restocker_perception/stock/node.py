# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/perception/lane_stock: the stock estimator as a ROS 2 node beside the lane observation node."""
# Subscribes to the wrist survey's LaneObservation (free depth, coverage, status: the evidence it
# already produces), the wrist depth and RGB frames and their camera_info, resolves the lane <-
# optical transform at the observation's stamp, reads the lane policy from the world state, and
# calls /identify_product on the rear product's crop. Publishes LaneStock. Nothing consumes it
# yet; LaneObservation and every existing consumer are untouched. With the identity sidecar absent
# (or the service not up) identity is UNAVAILABLE and the count is still reported.

from __future__ import annotations

import json
from pathlib import Path
import queue
import sys
import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from restocker_interfaces.msg import LaneObservation, LaneStock
from restocker_interfaces.srv import GetWorldState, IdentifyProduct
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from restocker_perception.identity.service import image_to_rgb
from restocker_perception.stock.config import load_stock_config
from restocker_perception.stock.depth_profile import Intrinsics
from restocker_perception.stock.estimator import (
    LaneEvidence,
    LaneStockEstimate,
    estimate_lane_stock,
)
from restocker_perception.stock.pipeline import estimate_from_frame

FRAME_SKEW_S = 0.25
MAX_CACHED_FRAMES = 8


def depth_to_metres(message: Image) -> np.ndarray:
    """A 32FC1 or 16UC1 (millimetre) depth image as float metres, row stride removed."""
    if message.encoding == "32FC1":
        rows = np.frombuffer(bytes(message.data), dtype=np.float32)
        return rows.reshape(message.height, message.step // 4)[:, : message.width].astype(
            np.float64
        )
    if message.encoding == "16UC1":
        rows = np.frombuffer(bytes(message.data), dtype=np.uint16)
        return rows.reshape(message.height, message.step // 2)[:, : message.width] / 1000.0
    raise ValueError(f"unsupported depth encoding {message.encoding!r}")


def fill_message(message: LaneStock, estimate: LaneStockEstimate) -> LaneStock:
    """Copy an estimate into a LaneStock message (header left to the caller)."""
    for name in (
        "lane_id",
        "state",
        "reason",
        "reason_detail",
        "expected_product_class",
        "has_expected_sku",
        "expected_sku",
        "free_depth_m",
        "occupied_depth_m",
        "usable_depth_m",
        "coverage",
        "product_length_m",
        "product_class_assumed",
        "count_estimate",
        "count_min",
        "count_max",
        "count_consistent",
        "visible_faces",
        "identity_channel_reachable",
        "face_skus",
        "face_identity_status",
        "named_count",
        "unverified_count",
        "composition",
        "misplaced",
        "misplaced_named_count",
        "misplaced_skus",
        "identity_registry_sha256",
        "identity_calibration_sha256",
    ):
        setattr(message, name, getattr(estimate, name))
    return message


class LaneStockNode(Node):
    def __init__(self, **node_options) -> None:
        super().__init__("lane_stock", **node_options)
        defaults = {
            "lane_observation_topic": "/perception/lane_observations",
            "lane_stock_topic": "/perception/lane_stock",
            "identify_service": "/identify_product",
            "depth_topic": "/wrist_camera/depth_image",
            "image_topic": "/wrist_camera/image",
            "camera_info_topic": "/wrist_camera/camera_info",
            "world_state_service": "/world_state/get_snapshot",
            "backend_name": "lane_stock_estimator",
            "backend_version": "1.0.0",
            "identify_timeout_sec": 2.5,
            "transform_timeout_sec": 0.2,
            "identity_mode": "safe",
            "config_path": "",
            # Optional JSON {product id: sku}: the identity channel names registry product ids
            # (the GTIN for a product that has one), a lane policy names SKUs. Empty compares
            # them as written. (Name kept from when the ids were GTINs.)
            "gtin_map_path": "",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.config = load_stock_config(get("config_path") or None)
        self.identity_mode = get("identity_mode")
        self.gtin_to_sku: dict[str, str] = {}
        if get("gtin_map_path"):
            self.gtin_to_sku = json.loads(Path(get("gtin_map_path")).read_text(encoding="utf-8"))
        self.depths: list[Image] = []
        self.images: list[Image] = []
        self.info: CameraInfo | None = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Image, get("depth_topic"), self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, get("image_topic"), self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, get("camera_info_topic"), self._on_info, qos_profile_sensor_data
        )
        self.identify = self.create_client(IdentifyProduct, get("identify_service"))
        self.world = self.create_client(GetWorldState, get("world_state_service"))
        self.publisher = self.create_publisher(LaneStock, get("lane_stock_topic"), 10)
        self.pending: queue.Queue = queue.Queue()
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()
        self.create_subscription(
            LaneObservation, get("lane_observation_topic"), self.pending.put, 10
        )
        self.get_logger().info("lane_stock ready")

    # --- caches -------------------------------------------------------------------------------
    def _on_depth(self, message: Image) -> None:
        self.depths = (self.depths + [message])[-MAX_CACHED_FRAMES:]

    def _on_image(self, message: Image) -> None:
        self.images = (self.images + [message])[-MAX_CACHED_FRAMES:]

    def _on_info(self, message: CameraInfo) -> None:
        self.info = message

    @staticmethod
    def _nearest(frames: list[Image], stamp: float, skew: float) -> Image | None:
        best = None
        for frame in frames:
            delta = abs(frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9 - stamp)
            if delta <= skew and (best is None or delta < best[0]):
                best = (delta, frame)
        return None if best is None else best[1]

    @staticmethod
    def _await(future, timeout_s: float):
        """Block the worker thread (never the executor's) until a service future completes."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout_s) else None

    def _work(self) -> None:
        while True:
            observation = self.pending.get()
            if observation is None:
                return
            try:
                self._estimate(observation)
            except Exception as error:  # noqa: BLE001 - one bad frame must not stop the node
                self.get_logger().error(f"lane stock estimate failed: {error!r}")

    # --- collaborators ---------------------------------------------------------------------------
    def _policy(self, lane_id: str) -> tuple[int, str | None]:
        """The lane policy (expected class, expected SKU) from the world state, or unknown."""
        if not self.world.service_is_ready():
            return 0, None
        request = GetWorldState.Request()
        response = self._await(self.world.call_async(request), 1.0)
        if response is None:
            return 0, None
        for lane in response.snapshot.lanes:
            if lane.id == lane_id:
                return int(lane.expected_product_class), (
                    lane.expected_sku if lane.has_expected_sku else None
                )
        return 0, None

    def _identify(self, crop: np.ndarray):
        """Ask the sidecar; None when the service is not there or does not answer in time."""
        if not self.identify.service_is_ready():
            return None
        request = IdentifyProduct.Request()
        message = Image()
        message.height, message.width = crop.shape[:2]
        message.encoding, message.step = "rgb8", crop.shape[1] * 3
        message.data = crop.astype(np.uint8).tobytes()
        request.crops = [message]
        response = self._await(
            self.identify.call_async(request),
            float(self.get_parameter("identify_timeout_sec").value),
        )
        if response is None:
            return None
        mapped = lambda gtin: self.gtin_to_sku.get(gtin, gtin)  # noqa: E731
        return {
            "status": int(response.status),
            "sidecar_available": bool(response.sidecar_available),
            "has_sku": bool(response.has_sku),
            "sku": mapped(response.sku),
            "reason": response.reason,
            "evidence_kinds": list(response.evidence_kinds),
            "evidence_product_ids": [mapped(g) for g in response.evidence_product_ids],
            "registry_sha256": response.registry_sha256,
            "calibration_sha256": response.calibration_sha256,
        }

    # --- the estimate ----------------------------------------------------------------------------
    def _estimate(self, observation: LaneObservation) -> None:
        stamp = observation.header.stamp.sec + observation.header.stamp.nanosec * 1e-9
        expected_class, expected_sku = self._policy(observation.lane_id)
        producer = {
            "ok": observation.status == LaneObservation.STATUS_OK,
            "coverage": float(observation.confidence),
            "free_depth_m": float(observation.available_depth_m),
            "obstructed": bool(observation.obstructed),
        }
        depth = self._nearest(self.depths, stamp, 0.001)
        image = self._nearest(self.images, stamp, FRAME_SKEW_S)
        if depth is None or self.info is None or not producer["ok"]:
            # No frame to read a profile from, or the producer refused. A refusal stays a refusal;
            # an observation with no matching frame still has a free depth, so it is counted from
            # that alone (no class check, no identity), which is what the policy class supports.
            estimate = estimate_lane_stock(
                LaneEvidence(
                    lane_id=observation.lane_id,
                    evidence_ok=producer["ok"],
                    coverage=producer["coverage"],
                    free_depth_m=producer["free_depth_m"],
                    obstructed=producer["obstructed"],
                    expected_class=expected_class,
                    expected_sku=expected_sku,
                ),
                self.config,
            )
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    observation.lane_id,
                    depth.header.frame_id,
                    depth.header.stamp,
                    timeout=Duration(
                        seconds=float(self.get_parameter("transform_timeout_sec").value)
                    ),
                )
            except tf2_ros.TransformException as error:
                self.get_logger().warning(f"no lane <- optical transform: {error}")
                return
            t, q = transform.transform.translation, transform.transform.rotation
            from restocker_perception.stock.depth_profile import lane_from_optical

            matrix = lane_from_optical([t.x, t.y, t.z, q.x, q.y, q.z, q.w], 0.0, (0.0, 0.0, 0.0))
            estimate, _ = estimate_from_frame(
                depth_to_metres(depth),
                None
                if image is None
                else image_to_rgb(
                    image.height, image.width, image.encoding, image.step, bytes(image.data)
                ),
                Intrinsics(self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]),
                matrix,
                self.config,
                lane_id=observation.lane_id,
                expected_class=expected_class,
                expected_sku=expected_sku,
                identify=self._identify,
                identity_mode=self.identity_mode,
                producer=producer,
            )
        message = fill_message(LaneStock(), estimate)
        message.header = observation.header
        message.backend_name = self.get_parameter("backend_name").value
        message.backend_version = self.get_parameter("backend_version").value
        self.publisher.publish(message)


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = LaneStockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pending.put(None)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
