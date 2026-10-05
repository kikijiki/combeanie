# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A lane that is obstructed rather than occupied is routed around."""
# Obstruction is now the only occupancy-shaped condition that makes a lane unusable outright. A
# lane that merely *holds* products is the ordinary destination for the next one of their class,
# ``test_restock_lane_routing_runtime`` puts two into one lane, and what refuses such a lane is
# the free depth behind its column, not the fact of it being occupied. ``obstructed`` is a
# different condition and, until the scenario this test launches existed, no shipped
# configuration could produce it: ``restocker_gazebo/lane_evidence.cpp`` marks a lane obstructed
# when a product's envelope overlaps the lane's usable volume without being contained in it, and
# every product every other scenario places is either in the stock tray or squarely inside a lane.
# So the flag selection reads at ``task_selection.cpp`` was covered by unit tests over fakes and
# was never once true in a running system.
#
# What this test adds over those unit tests is the two ends they cannot reach: that real Gazebo
# geometry and the real evidence pipeline actually produce ``obstructed``, and that the coordinator
# then delivers to the sibling lane through real physics, planning and control.
#
# The routing claim is a strict cost preference, not a tie-break. Selection scores rail travel
# only. With the bottle at x = -0.42 the delivery leg is 0.18 m to lane_02 against 1.02 m to
# lane_05, so lane_02 is cheaper by 0.84 m and is what an unobstructed shelf gets chosen every
# time, the control for that comparison is
# ``TaskSelectionTest.RoutesAroundAnObstructedLaneToItsCompatibleSibling``, which scores the same
# shelf with and without the flag. Here the obstruction is the only thing standing between the
# coordinator and the cheaper lane.

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
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct
from restocker_interfaces.msg import RestockCoordinatorStatus
from restocker_interfaces.srv import GetWorldState

# The scenario jams a small bottle in the mouth of lane_02, overlapping the usable volume and
# sticking out of the back of the shelf, and holds it there: the lane is a gravity-feed roller bed
# and a free product would run down it and settle inside the volume as an ordinary occupant. Small
# bottles are accepted by lane_02 and lane_05 only, so the bottle in the tray has exactly one
# admissible destination left.
OBSTRUCTED_LANE = "lane_02"
SIBLING_LANE = "lane_05"
STOCK_OBJECT = "sim:stock_small_bottle_01"
OBSTRUCTING_OBJECT = "sim:shelf_small_bottle_01"


@pytest.mark.launch_test
def generate_test_description():
    """Start the baseline against the obstructed-lane scenario."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            # Ground-truth path pins (Milestone 10 Stage 7 default switch):
            "object_observation_topic": "/perception/object_observations",
            "lane_observation_topic": "/perception/ground_truth/lane_observations",
            "tray_overview_perception": "false",
            "tray_confirm_perception": "false",
            "gui": "false",
            "rviz": "false",
            "planning_smoke": "false",
            "motion_enabled": "true",
            "controller_timeout": "30.0",
            "scenario_config": PathJoinSubstitution(
                [
                    FindPackageShare("restocker_gazebo"),
                    "config",
                    "lane_obstruction_products.yaml",
                ]
            ),
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestRestockObstructedLaneRuntime(unittest.TestCase):
    """Require a real obstruction, and a delivery to the sibling it forces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("restock_obstructed_lane_runtime_test")
        cls.latest_status = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_subscription = cls.node.create_subscription(
            RestockCoordinatorStatus,
            "/restock_action_coordinator/status",
            cls._on_status,
            status_qos,
        )
        cls.action_client = ActionClient(cls.node, RestockProduct, "/restock_product")
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_status(cls, status):
        cls.latest_status = status

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            result = predicate()
            if result:
                return result
        self.fail(f"timed out waiting for {description}")

    def _snapshot(self):
        self.assertTrue(
            self.world_state.wait_for_service(timeout_sec=30.0),
            "world state never offered its snapshot service",
        )
        future = self.world_state.call_async(GetWorldState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertIsNotNone(future.result(), "world state snapshot request did not complete")
        return future.result().snapshot

    def _admitted_snapshot(self):
        """Return a snapshot only once it carries admitted robot telemetry."""
        snapshot = self._snapshot()
        if snapshot.robot.telemetry_source_id and snapshot.robot.telemetry_revision > 0:
            return snapshot
        return None

    def _lanes(self, snapshot):
        return {lane.id: lane for lane in snapshot.lanes}

    def _describe(self, snapshot):
        """Render every lane's evidence, so a failure reports what was actually observed."""
        return " | ".join(
            f"{lane.id}: obstructed={lane.obstructed} contents={list(lane.contents)} "
            f"observed={list(lane.observed_source_object_ids)} "
            f"available_depth_m={lane.available_depth_m:.4f} "
            f"evidence_revision={lane.evidence_revision}"
            for lane in snapshot.lanes
        )

    def _occupied_lanes(self):
        return {lane.id: list(lane.contents) for lane in self._snapshot().lanes if lane.contents}

    def _assert_lane_02_is_obstructed_and_empty(self, snapshot, when):
        """Require the one condition this test exists for, distinguished from occupancy."""
        # Obstructed and occupied are separate fields fed by separate evidence, and reporting the
        # obstruction as occupancy, or the reverse, would satisfy the routing assertion below
        # for the wrong reason, so both are checked. The lane holds nothing by either measure:
        # ``contents`` is the world state's semantic inventory and
        # ``observed_source_object_ids`` is the fresh containment evidence behind it, and the
        # obstructing bottle appears in neither because it is not contained in the lane.
        lane = self._lanes(snapshot)[OBSTRUCTED_LANE]
        evidence = self._describe(snapshot)
        self.assertTrue(
            lane.obstructed,
            f"{OBSTRUCTED_LANE} must read as obstructed {when}: the scenario stands a bottle "
            f"across its front face. Observed {evidence}",
        )
        self.assertEqual(
            list(lane.contents), [], f"{OBSTRUCTED_LANE} must hold nothing {when}: {evidence}"
        )
        self.assertEqual(
            list(lane.observed_source_object_ids),
            [],
            f"{OBSTRUCTED_LANE} must carry no containment evidence {when}: an overlapping "
            f"product is an obstruction, not an occupant. Observed {evidence}",
        )
        self.assertGreater(
            lane.evidence_revision, 0, f"{OBSTRUCTED_LANE} was never observed {when}: {evidence}"
        )

    def test_a_product_whose_cheaper_lane_is_obstructed_reaches_its_sibling(self):
        self._spin_until(
            lambda: self.latest_status is not None and self.latest_status.admission_ready,
            120.0,
            "the coordinator to publish readiness",
        )
        # Readiness only says goals will be accepted; selection additionally needs an admitted
        # robot telemetry sample, which arrives independently.
        before = self._spin_until(
            self._admitted_snapshot,
            120.0,
            "the world state to admit robot telemetry",
        )

        lanes = self._lanes(before)
        self.assertEqual(len(lanes), 6, f"the shelf must carry six lanes, got {sorted(lanes)}")
        self._assert_lane_02_is_obstructed_and_empty(before, "before the transfer")
        # Exactly one lane is obstructed. A bottle placed a centimetre further across would also
        # obstruct a neighbour through a divider, which would still route to lane_05 and would
        # still let this test pass while measuring something coarser than it claims to.
        self.assertEqual(
            sorted(lane_id for lane_id, lane in lanes.items() if lane.obstructed),
            [OBSTRUCTED_LANE],
            f"the scenario must obstruct exactly one lane: {self._describe(before)}",
        )
        self.assertEqual(self._occupied_lanes(), {}, "the scenario must start with empty lanes")

        observed = {tracked.source_object_id: tracked for tracked in before.objects}
        for required in (STOCK_OBJECT, OBSTRUCTING_OBJECT):
            self.assertIn(required, observed, f"the obstruction scenario must stock {required}")
        bottle = observed[STOCK_OBJECT].id

        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=30.0),
            "the restock action server never became available",
        )

        # The goal names neither a product nor a lane: the coordinator picks both. Left to travel
        # cost alone it would pick lane_02, which is 0.84 m of delivery rail travel nearer than
        # lane_05; the obstruction is the only reason to prefer the sibling.
        goal = RestockProduct.Goal()
        goal.has_object_id = False
        goal.object_id = 0
        goal.has_lane_id = False
        goal.lane_id = ""
        goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=30.0)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, "the unaddressed goal was never answered")
        self.assertTrue(goal_handle.accepted, "the coordinator rejected the unaddressed goal")

        result_future = goal_handle.get_result_async()
        # Matches the sibling acceptance tests' budget: a full transfer is six planned segments,
        # two gripper moves and two attachment transactions under headless physics, and staying
        # above the coordinator's own total timeout means an overrun is reported with the
        # coordinator's reason rather than as a bare test timeout.
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=450.0)
        self.assertIsNotNone(result_future.result(), "the unaddressed goal produced no result")
        result = result_future.result().result
        self.assertEqual(
            result.status,
            RestockProduct.Result.STATUS_SUCCEEDED,
            f"the transfer failed: status={result.status} detail={result.detail}",
        )

        after = self._snapshot()
        self.assertEqual(
            {lane.id: list(lane.contents) for lane in after.lanes if lane.contents},
            {SIBLING_LANE: [bottle]},
            f"the bottle should have been routed to {SIBLING_LANE} and nothing else placed",
        )
        # The obstruction has to still be an obstruction afterwards. If the arm had displaced it,
        # or if the lane had quietly stopped reading as obstructed part way through, the routing
        # above would no longer be evidence about the flag.
        self._assert_lane_02_is_obstructed_and_empty(after, "after the transfer")
        self.assertFalse(after.robot.has_held_object, "the robot must end holding nothing")
        self.assertGreater(after.revision, before.revision)
