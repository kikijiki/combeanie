# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Autonomous product and lane choice, into a lane that already holds a product."""
# ``test_restock_manipulation_runtime`` addresses every goal explicitly. This test seeds one lane,
# then leaves both selectors unset so the system decides what to move and where. It checks two
# products in one lane: the second set down behind the first and carried into the back of it by
# the bed. A gravity-fed lane holds a column, so an occupied lane is a normal destination and
# selection skips a lane for lack of depth, not of emptiness. Routing around a lane that cannot
# take a product is covered by ``test_restock_obstructed_lane_runtime``.
#
# The two bottle goals are addressed because the second bottle sits at x = 0 in the tray, midway
# between lane_02 and lane_05. The cost is scored against the observed physics pose, which differs
# from the spawn pose by microns, so the tie is broken by which side of zero the bottle settles
# on (a run picked lane_05). The column is therefore asked for, and the coordinator's own choice
# is left to the third goal.
#
# Both selectors are left unset together, never one: this test only contrasts addressed goals
# (object and lane) with unaddressed ones. A lane without its object is still refused
# (``RequestedIdentityIncomplete``); the object-only form Card 037 added is not exercised here.

import math
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

# The scenario stocks two small bottles and one can. Small bottles are accepted by lane_02 and
# lane_05 only; both end up in lane_02 because this test addresses them there.
SEEDED_LANE = "lane_02"
SIBLING_LANE = "lane_05"
CAN_LANES = ("lane_01", "lane_04")
# Free depth one more small bottle takes off a packed lane: its diameter foreshortened by the
# 4-degree bed. Radius 0.034 m from product_collision_catalog.yaml, incline from
# workcell_geometry.yaml.
SMALL_BOTTLE_PITCH_M = 2.0 * 0.034 * math.cos(math.radians(4.0))


@pytest.mark.launch_test
def generate_test_description():
    """Start the baseline against the two-bottle routing scenario."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            # Ground-truth path pins (Milestone 10 Stage 7 default switch):
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
                    "lane_routing_products.yaml",
                ]
            ),
            # Two identical small bottles: nothing a camera measures says which the simulator calls
            # _01, and the simulated attachment adapter resolves identity straight to a Gazebo
            # model with no proximity check, so a wrong guess would latch the joint onto the other
            # bottle. Run on simulator ground truth; the settings are kept explicit even though
            # they match the defaults.
            "perception": "false",
            "object_observation_topic": "/perception/object_observations",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestRestockLaneRoutingRuntime(unittest.TestCase):
    """Require autonomous selection and a second product into an occupied lane."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("restock_lane_routing_runtime_test")
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

    def _restock(self, description, object_id=None, lane_id=None):
        """Drive one transfer, addressing it explicitly only when the caller supplies both IDs."""
        goal = RestockProduct.Goal()
        goal.has_object_id = object_id is not None
        goal.object_id = object_id if object_id is not None else 0
        goal.has_lane_id = lane_id is not None
        goal.lane_id = lane_id if lane_id is not None else ""
        goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=30.0)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, f"the {description} goal was never answered")
        self.assertTrue(goal_handle.accepted, f"the coordinator rejected the {description} goal")

        result_future = goal_handle.get_result_async()
        # Above the coordinator's own total timeout, so an overrun reports its reason. A transfer
        # is six planned segments, two gripper moves and two attachment transactions.
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=450.0)
        self.assertIsNotNone(result_future.result(), f"the {description} goal produced no result")
        result = result_future.result().result
        self.assertEqual(
            result.status,
            RestockProduct.Result.STATUS_SUCCEEDED,
            f"the {description} transfer failed: status={result.status} detail={result.detail}",
        )

    def _occupied_lanes(self):
        return {lane.id: list(lane.contents) for lane in self._snapshot().lanes if lane.contents}

    def _lane(self, lane_id):
        for lane in self._snapshot().lanes:
            if lane.id == lane_id:
                return lane
        self.fail(f"the world state does not carry {lane_id}")
        return None

    def _describe_world(self, label):
        """
        Print every lane's measured state and every product's pose.

        A lane's ledger (what the arm committed) and its geometry (where products are) can
        disagree; a failure message needs both side by side.
        """
        snapshot = self._snapshot()
        lines = [f"world state at {label}:"]
        for lane in snapshot.lanes:
            lines.append(
                f"  {lane.id}: contents={list(lane.contents)} "
                f"observed={list(lane.observed_source_object_ids)} "
                f"available_depth_m={lane.available_depth_m:.4f} obstructed={lane.obstructed}"
            )
        for tracked in snapshot.objects:
            position = tracked.pose.pose.position
            orientation = tracked.pose.pose.orientation
            lines.append(
                f"  {tracked.source_object_id} (id {tracked.id}): "
                f"xyz=({position.x:.4f}, {position.y:.4f}, {position.z:.4f}) "
                f"quat=({orientation.x:.4f}, {orientation.y:.4f}, "
                f"{orientation.z:.4f}, {orientation.w:.4f}) "
                f"tracking={tracked.tracking_state} grasp={tracked.grasp_state}"
            )
        print("\n".join(lines))

    def _assert_every_product_is_in_a_lane_that_accepts_it(self, occupied, admissible):
        """Require every placed product to sit in a lane its class allows."""
        # The order the coordinator moves the remaining products is a near-tie and is not
        # asserted; only that each product ends up in a lane that accepts it.
        for lane_id, contents in occupied.items():
            for object_id in contents:
                self.assertIn(
                    object_id, admissible, f"object {object_id} was not one the test stocked"
                )
                self.assertIn(
                    lane_id,
                    admissible[object_id],
                    f"object {object_id} was routed to {lane_id}, which does not accept it",
                )

    def test_a_lane_takes_a_second_product_and_the_coordinator_still_chooses_its_own(self):
        self._spin_until(
            lambda: self.latest_status is not None and self.latest_status.admission_ready,
            120.0,
            "the coordinator to publish readiness",
        )
        # Selection additionally needs an admitted robot telemetry sample.
        before = self._spin_until(
            self._admitted_snapshot,
            120.0,
            "the world state to admit robot telemetry",
        )
        self.assertEqual(self._occupied_lanes(), {}, "the scenario must start with empty lanes")
        empty_depth = self._lane(SEEDED_LANE).available_depth_m

        observed = {tracked.source_object_id: tracked for tracked in before.objects}
        for required in (
            "sim:stock_small_bottle_01",
            "sim:stock_small_bottle_02",
            "sim:stock_can_01",
        ):
            self.assertIn(required, observed, f"the routing scenario must stock {required}")
        first_bottle = observed["sim:stock_small_bottle_01"].id
        second_bottle = observed["sim:stock_small_bottle_02"].id
        can = observed["sim:stock_can_01"].id
        # Where each product may end up: the addressed bottles in one lane, the can in either of
        # its class's lanes.
        admissible = {
            first_bottle: (SEEDED_LANE,),
            second_bottle: (SEEDED_LANE,),
            can: CAN_LANES,
        }

        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=30.0),
            "the restock action server never became available",
        )

        # Seed lane_02 so it holds a product. The seeding and column goals are the only addressed
        # ones.
        self._restock("lane_02 seeding", object_id=first_bottle, lane_id=SEEDED_LANE)
        self.assertEqual(
            self._occupied_lanes(),
            {SEEDED_LANE: [first_bottle]},
            f"the seeding transfer should have filled {SEEDED_LANE} and nothing else",
        )
        one_deep = self._lane(SEEDED_LANE)
        self._describe_world("one bottle in the lane")
        self.assertLess(
            one_deep.available_depth_m,
            empty_depth,
            "a placed product must have taken depth off the lane",
        )

        # The second bottle, into the lane that already holds the first: selection's depth screen,
        # the reservation's room at the rear, the placement generator's containment and the
        # column-growth proof must all admit a non-empty destination.
        self._restock("column", object_id=second_bottle, lane_id=SEEDED_LANE)
        self._describe_world("two bottles in the lane")
        two_deep_contents = self._occupied_lanes()
        self._assert_every_product_is_in_a_lane_that_accepts_it(two_deep_contents, admissible)

        # The last goal names nothing. A shelved product has left the stock tray and selection
        # rejects it as outside stock, so this goal can only be the can.
        self._restock("unaddressed goal")
        final = self._occupied_lanes()
        self._assert_every_product_is_in_a_lane_that_accepts_it(final, admissible)

        # Both bottles are in the one lane in placement order: the bed ran the second into the
        # back of the first.
        self.assertEqual(
            final.get(SEEDED_LANE),
            [first_bottle, second_bottle],
            f"{SEEDED_LANE} should hold both bottles, first placed first, got {final}",
        )
        self.assertEqual(
            len(final.get(SEEDED_LANE, [])),
            2,
            f"{SEEDED_LANE} must hold two products, got {final}",
        )
        self.assertIn(
            final.get(CAN_LANES[0], final.get(CAN_LANES[1])),
            ([can],),
            f"the can should be alone in one of its own lanes, got {final}",
        )
        self.assertNotIn(
            SIBLING_LANE, final, f"{SIBLING_LANE} should have stayed empty, got {final}"
        )

        # The column-growth predicate already ran on the way in (the world state refuses the
        # second placement unless the column grew by a small bottle's pitch). The free depth is
        # printed, not asserted: products do not collide with each other in this simulation, so
        # both bottles slide to the front rail and rest at the same pose, and the settled column
        # is one bottle long whatever the count (0.7559 m free with one bottle, 0.7037 m right
        # after the second was placed, 0.7559 m once it slid through the first). This is a
        # simulation limitation, not a lane-policy one; the assertions below do not depend on it.
        two_deep = self._lane(SEEDED_LANE)
        self._describe_world("the end of the run")
        print(
            f"{SEEDED_LANE} free depth: {empty_depth:.4f} empty, "
            f"{one_deep.available_depth_m:.4f} with one bottle, "
            f"{two_deep.available_depth_m:.4f} with two; "
            f"one small bottle's pitch is {SMALL_BOTTLE_PITCH_M:.4f} m"
        )
        # The lane's evidence names both products, so the second is inside the lane volume.
        self.assertEqual(
            sorted(two_deep.observed_source_object_ids),
            ["sim:stock_small_bottle_01", "sim:stock_small_bottle_02"],
            f"{SEEDED_LANE} evidence should name both bottles, got {two_deep}",
        )
        self.assertLess(
            two_deep.available_depth_m,
            empty_depth,
            "a lane holding two products must report less free depth than an empty one",
        )
        self.assertFalse(two_deep.obstructed, f"{SEEDED_LANE} must not read as obstructed")

        snapshot = self._snapshot()
        self.assertFalse(snapshot.robot.has_held_object, "the robot must end holding nothing")
        self.assertGreater(snapshot.revision, before.revision)
