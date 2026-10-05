# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Shared world-state fixture for the autonomous campaign launch tests."""

import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from restocker_interfaces.msg import (
    RobotExecutionState,
    ShelfLane,
    TrackedObject,
    WorldStateSnapshot,
)
import yaml

LANE_CLASSES = (
    (ShelfLane.PRODUCT_CLASS_CAN, "SIM-CAN-STD"),
    (ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE, "SIM-BOTTLE-SMALL"),
    (ShelfLane.PRODUCT_CLASS_LARGE_BOTTLE, "SIM-BOTTLE-LARGE"),
)


def fixture_io_callback_group():
    """
    Return the callback group for fixture entities that answer while an action callback sleeps.

    Card 075: rclpy's node default group is MutuallyExclusive, so one sleeping
    ``goal_callback`` (S21's 6 s retreat admission hang) freezes every other entity on
    that group — including the world-state snapshot service. The campaign's
    ``measure_world`` is then bounded only by ``action_timeout_sec`` (3.0 in the walk)
    and publishes ``PHASE_BLOCKED`` ``world-state snapshot request timed out``, which
    S21's "every block is the rung-1 latch" assertion rejects. Action servers stay on
    the default group so scenario callbacks remain serialized, and so does the campaign
    status subscription (Card 077, 075 review note 2: on a Reentrant group two queued
    statuses can be appended out of arrival order, which the walk's window assertions
    assume cannot happen). Only the snapshot service moves here — it is the only entity
    on the failure path — and the fixture's call site is pinned by a test.
    """
    return ReentrantCallbackGroup()


def stamp(seconds):
    """Return a builtin_interfaces/Time at a float epoch, matching the node's clock."""
    return Time(nanoseconds=int(seconds * 1e9)).to_msg()


def pitch_by_sku():
    """Compute the catalogue pitch per SKU: 2 r cos(incline), as lane_deficit does."""
    share = get_package_share_directory("restocker_description")
    with Path(share, "config", "workcell_geometry.yaml").open() as stream:
        incline_deg = yaml.safe_load(stream)["shelf"]["lane_incline_deg"]
    with Path(share, "config", "product_collision_catalog.yaml").open() as stream:
        geometries = yaml.safe_load(stream)["geometries"]
    cosine = math.cos(math.radians(incline_deg))
    pitches = {}
    for geometry in geometries:
        pitches.setdefault(geometry["sku"], 2.0 * geometry["shape"]["radius_m"] * cosine)
    return pitches


# Where every tray product stands unless a scenario places it (world frame).
TRAY_POSITION = (-0.4, -0.8, 0.7)


class FakeWorld:
    """
    Hold the world-state snapshot the campaign measures against.

    Mutated only by the fakes. One re-entrant lock covers every mutation and every
    render: the fixture spins on a background executor, so a scenario's multi-field
    mutation must be atomic against the campaign's snapshot reads or a survey could
    observe half an update.
    """

    def __init__(self, pitches):
        self.lock = threading.RLock()
        self.pitches = pitches
        self.revision = 1
        self.next_object_id = 1
        self.lanes = {}
        for index in range(1, 7):
            product_class, sku = LANE_CLASSES[(index - 1) % 3]
            self.lanes[f"lane_{index:02d}"] = {
                "cls": product_class,
                "sku": sku,
                "target": 1,
                "depth": 1.0,
                "held": 0,
                "verified_age": None,
                "invalid": False,
                "evidence_revision": 0,
            }
        self.back = []

    def add_back(
        self, source, product_class, sku, observed_age=0.0, observed_at=None, position=None
    ):
        """
        Add a tray product.

        observed_age renders a stamp that moves with the clock (always that old);
        observed_at pins the last observation to one epoch instant (Card 069 streaks);
        position overrides the shared tray pose (Card 066: two products in one feed column).
        """
        with self.lock:
            self.back.append(
                {
                    "id": self.next_object_id,
                    "source": source,
                    "cls": product_class,
                    "sku": sku,
                    "observed_age": observed_age,
                    "observed_at": observed_at,
                    "orientation": TrackedObject.ORIENTATION_UPRIGHT,
                    "position": position or TRAY_POSITION,
                }
            )
            self.next_object_id += 1

    def observe(self, source):
        """Re-observe a tray product now, as a confirm view admitting it would."""
        with self.lock:
            for entry in self.back:
                if entry["source"] == source:
                    entry["observed_at"] = None
                    entry["observed_age"] = 0.0
            self.revision += 1

    def survey_lane(self, lane_id):
        with self.lock:
            lane = self.lanes[lane_id]
            lane["verified_age"] = 0.0
            lane["invalid"] = False
            self.revision += 1
            lane["evidence_revision"] = self.revision

    def expire_lane(self, lane_id, age_seconds=120.0):
        """Age one lane's evidence past the horizon, as time passing during a long step does."""
        with self.lock:
            self.lanes[lane_id]["verified_age"] = age_seconds

    def apply_transfer(self, lane_id, source):
        with self.lock:
            lane = self.lanes[lane_id]
            lane["held"] += 1
            lane["verified_age"] = 0.0
            lane["invalid"] = False
            self.revision += 1
            lane["evidence_revision"] = self.revision
            self.back = [entry for entry in self.back if entry["source"] != source]

    def admit(self, source, product_class, sku):
        """Apply what the confirm duty's admitted observation does to the snapshot."""
        with self.lock:
            previous = [entry for entry in self.back if entry["source"] == source]
            self.back = [entry for entry in self.back if entry["source"] != source]
            admitted_id = self.next_object_id
            self.back.append(
                {
                    "id": admitted_id,
                    "source": source,
                    "cls": product_class,
                    "sku": sku,
                    "observed_age": 0.0,
                    "position": previous[0].get("position", TRAY_POSITION)
                    if previous
                    else TRAY_POSITION,
                }
            )
            self.next_object_id += 1
            self.revision += 1
            return admitted_id

    def snapshot(self):
        with self.lock:
            now = time.time()
            message = WorldStateSnapshot()
            message.header.frame_id = "world"
            message.header.stamp = stamp(now)
            message.revision = self.revision
            for lane_id in sorted(self.lanes):
                lane = self.lanes[lane_id]
                lane_message = ShelfLane()
                lane_message.id = lane_id
                lane_message.expected_product_class = lane["cls"]
                lane_message.has_expected_sku = True
                lane_message.expected_sku = lane["sku"]
                lane_message.target_count = lane["target"]
                lane_message.depth_m = lane["depth"]
                pitch = self.pitches[lane["sku"]]
                lane_message.available_depth_m = max(0.0, lane["depth"] - lane["held"] * pitch)
                lane_message.evidence_invalidated = lane["invalid"]
                lane_message.evidence_invalidated_at = stamp(now)
                lane_message.evidence_revision = lane["evidence_revision"]
                verified = now if lane["verified_age"] is None else now - lane["verified_age"]
                lane_message.last_verified = stamp(verified)
                lane_message.revision = self.revision
                message.lanes.append(lane_message)
            for entry in self.back:
                object_message = TrackedObject()
                object_message.id = entry["id"]
                object_message.source_object_id = entry["source"]
                object_message.product_class = entry["cls"]
                object_message.has_sku = True
                object_message.sku = entry["sku"]
                position = entry.get("position", TRAY_POSITION)
                object_message.pose.pose.position.x = position[0]
                object_message.pose.pose.position.y = position[1]
                object_message.pose.pose.position.z = position[2]
                object_message.pose.pose.orientation.w = 1.0
                for index in (0, 7, 14):
                    object_message.pose.covariance[index] = 1.0e-8
                object_message.orientation = entry.get(
                    "orientation", TrackedObject.ORIENTATION_UPRIGHT
                )
                object_message.tracking_state = TrackedObject.TRACKING_TRACKED
                object_message.grasp_state = TrackedObject.GRASP_FREE
                if entry.get("observed_at") is not None:
                    observed = stamp(entry["observed_at"])
                else:
                    observed = stamp(now - entry["observed_age"])
                object_message.observation_time = observed
                object_message.transition_time = observed
                object_message.revision = self.revision
                message.objects.append(object_message)
            robot = message.robot
            robot.revision = self.revision
            robot.grasp_center_from_held_object.orientation.w = 1.0
            robot.task_phase = RobotExecutionState.TASK_IDLE
            robot.fault_state = RobotExecutionState.FAULT_NONE
            return message
