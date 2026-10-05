# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 075: the fixture's world-state service must answer while a goal_callback sleeps.

S21 in test_autonomous_restock_campaign delays the retreat admission response by sleeping
inside goal_callback for 6 s. rclpy's default callback group is MutuallyExclusive, so every
other entity on that group is frozen for the whole sleep. The campaign's measure_world wait
is only action_timeout_sec (3.0 s in the walk), so a snapshot service left on the default
group cannot answer in time and the campaign publishes PHASE_BLOCKED
"world-state snapshot request timed out" — which S21's assertion rejects.

This test pins the wiring the campaign fixture uses (fixture_io_callback_group) the same way:
a hanging admission callback must not delay a world-state snapshot round trip. It exercises the
helper with its own nodes; the fixture's actual call site — and the status subscription staying
on the node default group — is pinned inside `test_autonomous_restock_campaign` (Card 077).
"""

import threading
import time
import unittest

from campaign_loop_support import fixture_io_callback_group
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from restocker_interfaces.action import SurveyViewpoint
from restocker_interfaces.srv import GetWorldState

ADMISSION_HANG_SEC = 4.0
# Comfortably inside the hang, far below action_timeout_sec's 3.0 s campaign budget
# once the hang would otherwise block the service for the full 4.0 s.
SNAPSHOT_BUDGET_SEC = 2.0


def _wait_future(future, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.02)
    return None


class TestFixtureWorldStateResponsiveness(unittest.TestCase):
    """The snapshot service answers while a sibling goal_callback is sleeping."""

    def test_world_state_answers_during_admission_hang(self):
        rclpy.init()
        server_node = rclpy.create_node("card075_fixture_server")
        client_node = rclpy.create_node("card075_fixture_client")
        io_group = fixture_io_callback_group()

        def admission_hang(_goal_request):
            time.sleep(ADMISSION_HANG_SEC)
            return GoalResponse.ACCEPT

        def execute(_goal_handle):
            from restocker_interfaces.action import SurveyViewpoint as SV

            result = SV.Result()
            result.outcome = SV.Result.OUTCOME_ARRIVED
            result.execution_reached_terminal_stop = True
            _goal_handle.succeed()
            return result

        def snapshot(_request, response):
            response.snapshot.revision = 1
            return response

        # Action server on the node default group — same as the campaign fixture.
        ActionServer(
            server_node,
            SurveyViewpoint,
            "/card075/test/survey_viewpoint",
            execute_callback=execute,
            goal_callback=admission_hang,
        )
        # World-state service through the shared fixture helper — the wiring under test.
        server_node.create_service(
            GetWorldState,
            "/card075/test/get_snapshot",
            snapshot,
            callback_group=io_group,
        )

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(server_node)
        executor.add_node(client_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            action_client = ActionClient(
                client_node, SurveyViewpoint, "/card075/test/survey_viewpoint"
            )
            snapshot_client = client_node.create_client(
                GetWorldState, "/card075/test/get_snapshot"
            )
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not (
                action_client.server_is_ready() and snapshot_client.service_is_ready()
            ):
                time.sleep(0.02)
            self.assertTrue(action_client.server_is_ready(), "retreat server not ready")
            self.assertTrue(snapshot_client.service_is_ready(), "snapshot service not ready")

            # Start the admission hang, then request a snapshot while it sleeps.
            action_client.send_goal_async(SurveyViewpoint.Goal(station="tray_1"))
            time.sleep(0.3)
            self.assertLess(
                SNAPSHOT_BUDGET_SEC,
                ADMISSION_HANG_SEC,
                "the budget must sit inside the hang for this test to mean anything",
            )
            started = time.monotonic()
            response = _wait_future(
                snapshot_client.call_async(GetWorldState.Request()), SNAPSHOT_BUDGET_SEC
            )
            elapsed = time.monotonic() - started
            self.assertIsNotNone(
                response,
                f"world-state snapshot did not answer within {SNAPSHOT_BUDGET_SEC:.1f}s "
                f"while goal_callback was sleeping {ADMISSION_HANG_SEC:.1f}s "
                f"(waited {elapsed:.3f}s) — the service shares a MutuallyExclusive "
                "callback group with the hanging admission callback",
            )
            self.assertEqual(response.snapshot.revision, 1)
        finally:
            executor.shutdown()
            spin_thread.join(timeout=5.0)
            server_node.destroy_node()
            client_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
