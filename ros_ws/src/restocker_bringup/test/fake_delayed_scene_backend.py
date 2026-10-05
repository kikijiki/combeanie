#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Controllable world-state and MoveIt services for projector fencing tests."""

from copy import deepcopy
import json
from threading import Event, Lock
import time

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from restocker_interfaces.msg import TrackedObject
from restocker_interfaces.srv import GetWorldState
from std_srvs.srv import Trigger


class DelayedSceneBackend(Node):
    """Hold the first apply response while continuing to serve control calls."""

    def __init__(self):
        # Named after the service prefix so its parameter services land on
        # /fake_scene_backend/set_parameters like the explicit ones below.
        super().__init__("fake_scene_backend")
        self._callbacks = ReentrantCallbackGroup()
        self._lock = Lock()
        self._release_apply = Event()
        self._apply_pending = False
        self._snapshot_requests = 0
        self._get_scene_requests = 0
        self._apply_requests = 0
        self._scene = PlanningScene()
        self._scene.is_diff = False
        # Wall-clock delay inserted into every snapshot reply, changed at runtime through the
        # node's ordinary set_parameters service. It stands in for a world state whose executor
        # is starved: the service is alive and answers, just later than the projector's nominal
        # deadline.
        self.declare_parameter("snapshot_delay_sec", 0.0)
        # The apply fence is what test_projector_apply_timeout_fence exercises; tests that only
        # need a certifying scene switch it off, because a held apply stops the projector in
        # ApplyScene for ever.
        self.declare_parameter("hold_first_apply", True)
        # When non-negative, every snapshot carries one free, tracked can outside every lane whose
        # last observation is this many seconds old (Card 071: an aged product must stay an
        # obstacle, not fail the projection). Negative means an empty world.
        self.declare_parameter("aged_product_age_sec", -1.0)

        self.create_service(
            GetWorldState,
            "/world_state/get_snapshot",
            self._get_snapshot,
            callback_group=self._callbacks,
        )
        self.create_service(
            GetPlanningScene,
            "/get_planning_scene",
            self._get_scene,
            callback_group=self._callbacks,
        )
        self.create_service(
            ApplyPlanningScene,
            "/apply_planning_scene",
            self._apply_scene,
            callback_group=self._callbacks,
        )
        self.create_service(
            Trigger,
            "/fake_scene_backend/query",
            self._query,
            callback_group=self._callbacks,
        )
        self.create_service(
            Trigger,
            "/fake_scene_backend/release_apply",
            self._release,
            callback_group=self._callbacks,
        )

    def _get_snapshot(self, _request, response):
        with self._lock:
            self._snapshot_requests += 1
        delay = float(self.get_parameter("snapshot_delay_sec").value)
        if delay > 0.0:
            # Counted before the wait: a client that abandons and re-requests shows up here as
            # extra requests, a client that holds its request shows up as one. Sliced so a
            # shutdown during a long injected delay still lets this process exit on SIGINT.
            remaining = delay
            while remaining > 0.0 and rclpy.ok():
                step = min(0.1, remaining)
                time.sleep(step)
                remaining -= step
        response.snapshot.header.stamp = self.get_clock().now().to_msg()
        response.snapshot.header.frame_id = "world"
        response.snapshot.revision = 10
        age = float(self.get_parameter("aged_product_age_sec").value)
        if age >= 0.0:
            response.snapshot.objects = [self._aged_product(age)]
        return response

    def _aged_product(self, age_sec):
        observed = (self.get_clock().now() - Duration(seconds=age_sec)).to_msg()
        product = TrackedObject()
        product.id = 1
        product.source_object_id = "sim:aged_can"
        product.product_class = TrackedObject.PRODUCT_CLASS_CAN
        product.has_sku = True
        product.sku = "SIM-CAN-STD"
        product.pose.pose.position.x = 0.5
        product.pose.pose.position.y = -0.5
        product.pose.pose.position.z = 0.8
        product.pose.pose.orientation.w = 1.0
        product.orientation = TrackedObject.ORIENTATION_UPRIGHT
        product.tracking_state = TrackedObject.TRACKING_TRACKED
        product.grasp_state = TrackedObject.GRASP_FREE
        product.observation_time = observed
        product.transition_time = observed
        product.revision = 1
        return product

    def _get_scene(self, _request, response):
        with self._lock:
            self._get_scene_requests += 1
            response.scene = deepcopy(self._scene)
        return response

    def _apply_scene(self, request, response):
        with self._lock:
            self._apply_requests += 1
            hold = bool(self.get_parameter("hold_first_apply").value)
            if hold:
                self._apply_pending = True
        if hold:
            self._release_apply.wait()
        with self._lock:
            objects = {item.id: deepcopy(item) for item in self._scene.world.collision_objects}
            for item in request.scene.world.collision_objects:
                if item.operation == CollisionObject.REMOVE:
                    objects.pop(item.id, None)
                else:
                    applied = deepcopy(item)
                    applied.operation = CollisionObject.ADD
                    objects[item.id] = applied
            self._scene.world.collision_objects = list(objects.values())
            self._apply_pending = False
        response.success = True
        return response

    def _query(self, _request, response):
        with self._lock:
            response.success = self._apply_pending
            response.message = json.dumps(
                {
                    "snapshot_requests": self._snapshot_requests,
                    "get_scene_requests": self._get_scene_requests,
                    "apply_requests": self._apply_requests,
                    "snapshot_delay_sec": float(self.get_parameter("snapshot_delay_sec").value),
                },
                sort_keys=True,
            )
        return response

    def _release(self, _request, response):
        with self._lock:
            response.success = self._apply_pending
        response.message = "released pending apply" if response.success else "no apply pending"
        if response.success:
            self._release_apply.set()
        return response


def main():
    """Run the fake backend with enough threads for its blocked apply callback."""
    rclpy.init()
    node = DelayedSceneBackend()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
