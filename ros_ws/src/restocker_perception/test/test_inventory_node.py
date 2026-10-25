# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""/perception/inventory and /query_inventory over a real ROS graph with a fake world state."""
# One process on a private DDS domain. A fake lane stock estimator publishes LaneStock, a fake
# world state serves the snapshot, and the node must publish an Inventory and answer
# QueryInventory. The aggregation itself is covered without ROS in test_inventory.py.

import os
from pathlib import Path
import random
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from builtin_interfaces.msg import Time  # noqa: E402,I100,I101
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from restocker_interfaces.msg import (  # noqa: E402
    Inventory,
    InventoryLane,
    LaneStock,
    ObjectObservation,
    ShelfLane,
    TrackedObject,
)
from restocker_interfaces.srv import GetWorldState, QueryInventory  # noqa: E402
from restocker_perception.inventory.node import InventoryNode  # noqa: E402


@pytest.fixture(scope="module")
def ros_context():
    context = rclpy.Context()
    domain = random.Random(os.getpid() + 11).randint(120, 200)
    rclpy.init(context=context, domain_id=domain)
    os.environ["ROS_DOMAIN_ID"] = str(domain)  # the operator tool joins the same graph
    yield context
    rclpy.shutdown(context=context)


def stock(lane, sku, count, stamp, names=()):
    message = LaneStock()
    message.header.stamp = stamp
    message.lane_id, message.state = lane, LaneStock.STATE_ESTIMATED
    message.has_expected_sku, message.expected_sku = True, sku
    message.count_estimate = message.count_min = message.count_max = count
    message.face_skus = list(names)
    message.face_identity_status = [LaneStock.IDENTITY_NAMED] * len(names)
    message.named_count = len(names)
    return message


class Graph:
    def __init__(self, context, ledger):
        self.context = context
        self.fake = rclpy.create_node("fake_upstream", context=context)
        self.stock_pub = self.fake.create_publisher(LaneStock, "/perception/lane_stock", 10)
        self.tray_pub = self.fake.create_publisher(ObjectObservation, "/tray", 10)
        self.ledger = ledger
        self.fake.create_service(GetWorldState, "/world_state/get_snapshot", self._world)
        self.node = InventoryNode(
            context=context,
            parameter_overrides=[
                rclpy.parameter.Parameter("stale_after_s", rclpy.Parameter.Type.DOUBLE, 900.0),
                rclpy.parameter.Parameter(
                    "tray_observation_topic", rclpy.Parameter.Type.STRING, "/tray"
                ),
                rclpy.parameter.Parameter("publish_period_s", rclpy.Parameter.Type.DOUBLE, 0.1),
                rclpy.parameter.Parameter(
                    "world_refresh_period_s", rclpy.Parameter.Type.DOUBLE, 0.1
                ),
            ],
        )
        self.client = self.fake.create_client(QueryInventory, "/query_inventory")
        self.published = []
        self.fake.create_subscription(
            Inventory, "/perception/inventory", self.published.append, 10
        )
        self.executor = MultiThreadedExecutor(context=context, num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.fake)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _world(self, _request, response):
        lanes, objects, ident = [], [], 1
        for lane_id, (sku, count) in self.ledger.items():
            lane = ShelfLane()
            lane.id, lane.has_expected_sku, lane.expected_sku = lane_id, True, sku
            for _ in range(count):
                tracked = TrackedObject()
                tracked.id, tracked.has_sku, tracked.sku = ident, True, sku
                tracked.source_object_id = f"sim:{ident}"
                objects.append(tracked)
                lane.contents.append(ident)
                ident += 1
            lanes.append(lane)
        loose = TrackedObject()
        loose.id, loose.has_sku, loose.sku, loose.source_object_id = ident, True, "GEN-009", "t"
        objects.append(loose)
        response.snapshot.lanes, response.snapshot.objects = lanes, objects
        response.snapshot.revision = 5
        return response

    def query(self, **fields):
        assert self.client.wait_for_service(timeout_sec=10.0)
        request = QueryInventory.Request()
        for key, value in fields.items():
            setattr(request, key, value)
        future = self.client.call_async(request)
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            time.sleep(0.02)
        return future.result()

    def wait(self, predicate, seconds=10.0):
        deadline = time.time() + seconds
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.fake.destroy_node()


def now_stamp(graph):
    nanoseconds = graph.node.get_clock().now().nanoseconds
    return Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)


def test_publishes_an_inventory_and_answers_a_sku_query(ros_context):
    graph = Graph(ros_context, {"lane_01": ("GEN-001", 3), "lane_02": ("GEN-002", 2)})
    try:
        graph.stock_pub.publish(
            stock("lane_01", "GEN-001", 3, now_stamp(graph), names=("GEN-001",))
        )
        assert graph.wait(
            lambda: any(
                m.world_available and any(x.lane_id == "lane_01" for x in m.lanes)
                for m in graph.published
            )
        )
        message = graph.published[-1]
        by_lane = {x.lane_id: x for x in message.lanes}
        assert by_lane["lane_01"].observation == InventoryLane.OBSERVATION_FRESH
        assert by_lane["lane_01"].confirmed_count == 1
        assert by_lane["lane_02"].observation == InventoryLane.OBSERVATION_NEVER
        assert message.tray_source == Inventory.TRAY_SOURCE_WORLD_STATE and message.tray_count == 1
        assert message.world_revision == 5

        reply = graph.query(sku="GEN-001")
        assert reply.ok and reply.count_known and reply.count_estimate == 3
        assert reply.confirmed_count == 1 and reply.complete
        assert "GEN-001" in reply.explanation

        reply = graph.query(sku="GEN-002")
        assert reply.ok and not reply.complete and "lane_02" in reply.explanation
        assert list(reply.inventory.skus[0].unread_lanes) == ["lane_02"]

        reply = graph.query(lane_id="lane_09")
        assert not reply.ok and "lane_09" in reply.message

        reply = graph.query(
            sku="GEN-001", min_confidence=QueryInventory.Request.CONFIDENCE_CONFIRMED
        )
        assert reply.count_estimate == 1 and reply.excluded_count == 2
    finally:
        graph.close()


def test_an_old_reading_is_stale_and_a_count_mismatch_is_a_discrepancy(ros_context):
    graph = Graph(ros_context, {"lane_01": ("GEN-001", 5), "lane_02": ("GEN-002", 2)})
    try:
        old = now_stamp(graph)
        old.sec -= 5000
        graph.stock_pub.publish(stock("lane_02", "GEN-002", 2, old))
        graph.stock_pub.publish(stock("lane_01", "GEN-001", 3, now_stamp(graph)))
        assert graph.wait(
            lambda: any(
                {x.lane_id for x in m.lanes} == {"lane_01", "lane_02"}
                and m.world_available
                and m.lanes_stale == 1
                for m in graph.published
            )
        )
        by_lane = {x.lane_id: x for x in graph.published[-1].lanes}
        assert by_lane["lane_02"].observation == InventoryLane.OBSERVATION_STALE
        assert by_lane["lane_02"].age_s > 4000
        assert (
            by_lane["lane_01"].count_discrepancy == InventoryLane.COUNT_DISCREPANCY_OBSERVED_LOWER
        )
        assert by_lane["lane_01"].count_delta == -2
        assert graph.published[-1].discrepancy_count == 1
        reply = graph.query(sku="GEN-002", max_age_s=100000.0)
        assert reply.count_estimate == 2  # the caller's horizon admits the old reading
        assert not graph.query(sku="GEN-002").complete
    finally:
        graph.close()


def test_a_tray_observation_with_identity_wins_over_the_world_state(ros_context):
    graph = Graph(ros_context, {"lane_01": ("GEN-001", 1)})
    try:
        stamp = now_stamp(graph)
        for index, (state, sku) in enumerate(
            [
                (ObjectObservation.IDENTITY_IDENTIFIED, "GEN-004"),
                (ObjectObservation.IDENTITY_UNKNOWN_PRODUCT, ""),
            ]
        ):
            observation = ObjectObservation()
            observation.header.stamp = stamp
            observation.source_object_id = f"tray:{index}"
            observation.status = ObjectObservation.STATUS_OK
            observation.identity_state = state
            observation.has_sku, observation.sku = bool(sku), sku
            graph.tray_pub.publish(observation)
        assert graph.wait(
            lambda: any(
                m.tray_source == Inventory.TRAY_SOURCE_PERCEPTION and m.world_available
                for m in graph.published
            )
        )
        message = graph.published[-1]
        assert message.tray_count == 2
        sku = {x.sku: x for x in message.skus}
        assert sku["GEN-004"].confirmed_count == 1 and sku["GEN-004"].tray_count == 1
        # The world state's own tray item is compared, not counted, once perception has the tray.
        assert sku["GEN-009"].reported_count == 0 and sku["GEN-009"].world_count == 1
    finally:
        graph.close()


def test_operator_tool_answers_how_many_in_stock(ros_context, capsys):
    from restocker_perception.inventory import cli

    graph = Graph(ros_context, {"lane_01": ("GEN-001", 3)})
    try:
        graph.stock_pub.publish(stock("lane_01", "GEN-001", 3, now_stamp(graph)))
        assert graph.wait(
            lambda: any(m.world_available and m.lanes_fresh == 1 for m in graph.published)
        )
        assert cli.main(["query", "--sku", "GEN-001"]) == 0
        out = capsys.readouterr().out
        assert "3 of GEN-001 in stock" in out and "complete" in out
        assert cli.main(["show"]) == 0
        assert "lane_01" in capsys.readouterr().out
        assert cli.main(["query", "--lane", "lane_77"]) == 1
    finally:
        graph.close()
