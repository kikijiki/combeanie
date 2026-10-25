# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/perception/inventory and /query_inventory: the inventory as a ROS 2 node."""
# Subscribes to LaneStock (the lane stock estimator), optionally to a tray observation topic whose
# ObjectObservations carry the identity state (CMB-113), and polls the world state's snapshot
# service; publishes Inventory and serves QueryInventory. Nothing is mutated upstream. With no
# LaneStock yet every lane is "never read"; with no world state the tray is not covered and no
# discrepancy is computed, and the inventory says so instead of reporting zeros.

from __future__ import annotations

import sys
import threading

import rclpy
from rclpy.node import Node
from restocker_interfaces.msg import Inventory, LaneStock, ObjectObservation
from restocker_interfaces.srv import GetWorldState, QueryInventory

from restocker_perception.inventory.messages import inventory_message
from restocker_perception.inventory.model import (
    IDENTITY_STATE_IDENTIFIED,
    IDENTITY_STATE_UNKNOWN_PRODUCT,
    InventoryConfig,
    LaneReading,
    TrayItem,
    TrayReading,
    WorldView,
)
from restocker_perception.inventory.query import query_inventory
from restocker_perception.inventory.world import world_view_from_snapshot


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def tray_item(observation: ObjectObservation) -> TrayItem | None:
    """One tray observation as an inventory item, or None when it is not a usable detection."""
    if observation.status != ObjectObservation.STATUS_OK:
        return None
    if observation.identity_state == IDENTITY_STATE_IDENTIFIED and observation.has_sku:
        return TrayItem(observation.sku, identified=True)
    if observation.identity_state == IDENTITY_STATE_UNKNOWN_PRODUCT:
        return TrayItem(None)
    # Not assessed: a detection whose name is the manifest's, not an identity channel's.
    return TrayItem(observation.sku if observation.has_sku else None, identified=False)


class InventoryNode(Node):
    def __init__(self, **node_options) -> None:
        super().__init__("inventory", **node_options)
        defaults = {
            "lane_stock_topic": "/perception/lane_stock",
            "inventory_topic": "/perception/inventory",
            "query_service": "/query_inventory",
            "world_state_service": "/world_state/get_snapshot",
            # Empty: no perception tray reading, the tray comes from the world state.
            "tray_observation_topic": "",
            # Declared horizon after which a lane's reading is stale (deployment decision).
            "stale_after_s": 900.0,
            "publish_period_s": 1.0,
            "world_refresh_period_s": 1.0,
            # A cached world snapshot older than this is not used.
            "world_max_age_s": 10.0,
            # Tray observations within this of the newest belong to one reading.
            "tray_window_s": 2.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.config = InventoryConfig(stale_after_s=float(get("stale_after_s")))
        self.lock = threading.Lock()
        self.readings: dict[str, LaneReading] = {}
        self.world: WorldView | None = None
        self.world_stamp_s = 0.0
        self.tray_seen: dict[str, tuple[float, TrayItem]] = {}
        self.create_subscription(LaneStock, get("lane_stock_topic"), self._on_lane_stock, 10)
        if get("tray_observation_topic"):
            self.create_subscription(
                ObjectObservation, get("tray_observation_topic"), self._on_tray, 50
            )
        self.world_client = self.create_client(GetWorldState, get("world_state_service"))
        self.publisher = self.create_publisher(Inventory, get("inventory_topic"), 10)
        self.create_service(QueryInventory, get("query_service"), self._on_query)
        self.create_timer(float(get("world_refresh_period_s")), self._refresh_world)
        self.create_timer(float(get("publish_period_s")), self._publish)
        self.get_logger().info("inventory ready")

    # --- inputs ----------------------------------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_lane_stock(self, message: LaneStock) -> None:
        stamp = stamp_seconds(message.header.stamp) or self._now_s()
        with self.lock:
            kept = self.readings.get(message.lane_id)
            if kept is None or stamp >= kept.stamp_s:
                self.readings[message.lane_id] = LaneReading.from_stock(message, stamp)

    def _on_tray(self, message: ObjectObservation) -> None:
        item = tray_item(message)
        stamp = stamp_seconds(message.header.stamp) or self._now_s()
        with self.lock:
            if item is None:
                self.tray_seen.pop(message.source_object_id, None)
            else:
                self.tray_seen[message.source_object_id] = (stamp, item)

    def _refresh_world(self) -> None:
        if not self.world_client.service_is_ready():
            return
        request = GetWorldState.Request()
        future = self.world_client.call_async(request)

        def done(result) -> None:
            try:
                response = result.result()
            except Exception as error:  # noqa: BLE001 - a lost reply must not stop the node
                self.get_logger().warning(f"world state refresh failed: {error!r}")
                return
            view = world_view_from_snapshot(response.snapshot)
            with self.lock:
                self.world, self.world_stamp_s = view, self._now_s()

        future.add_done_callback(done)

    # --- state -----------------------------------------------------------------------------------
    def _state(self) -> tuple[dict[str, LaneReading], WorldView | None, TrayReading | None, float]:
        now = self._now_s()
        window = float(self.get_parameter("tray_window_s").value)
        with self.lock:
            readings = dict(self.readings)
            world = self.world
            if world is not None and now - self.world_stamp_s > float(
                self.get_parameter("world_max_age_s").value
            ):
                world = None
            tray = None
            if self.tray_seen:
                newest = max(stamp for stamp, _ in self.tray_seen.values())
                items = [
                    item for stamp, item in self.tray_seen.values() if newest - stamp <= window
                ]
                tray = TrayReading(newest, items)
        return readings, world, tray, now

    # --- outputs ---------------------------------------------------------------------------------
    def _publish(self) -> None:
        readings, world, tray, now = self._state()
        answer = query_inventory(readings, world, tray, now, self.config)
        if answer.view is not None:
            self.publisher.publish(inventory_message(answer.view))

    def _on_query(self, request: QueryInventory.Request, response: QueryInventory.Response):
        readings, world, tray, now = self._state()
        answer = query_inventory(
            readings,
            world,
            tray,
            now,
            self.config,
            sku=request.sku,
            lane_id=request.lane_id,
            min_confidence=int(request.min_confidence),
            max_age_s=float(request.max_age_s),
        )
        response.ok = answer.ok
        response.message = answer.message
        response.count_known = answer.count_known
        response.count_min = answer.count_min
        response.count_max = answer.count_max
        response.count_estimate = answer.count_estimate
        response.confirmed_count = answer.confirmed_count
        response.excluded_count = answer.excluded_count
        response.complete = answer.complete
        response.explanation = answer.explanation
        if answer.view is not None:
            response.inventory = inventory_message(answer.view)
        return response


def main(argv=None) -> int:
    rclpy.init(args=argv if argv is not None else sys.argv)
    node = InventoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
