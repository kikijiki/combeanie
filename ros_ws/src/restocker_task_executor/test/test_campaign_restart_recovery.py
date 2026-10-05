# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1 (CMB-SPEC-13): restart recovery and an unproven restock terminal.

Unresolved-motion state is in memory only, so a restarted campaign reports PHASE_RECOVERING and
sends nothing until an operator sets restart_acknowledged=true. After the acknowledgment a
restock terminal that failed without non-start or stop evidence leaves the attempt unresolved:
the campaign sends nothing more even though the world offers fresh work.
"""

import time

import pytest
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

PREFIX = "/test/campaign_restart_recovery"
NODE_NAME = "autonomous_restock_campaign_restart_subject"
Status = AutonomousRestockCampaignStatus


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign unacknowledged, as after any process restart."""
    return support.make_description(PREFIX, NODE_NAME, restart_acknowledged=False)


class TestRestartRecovery(support.UnresolvedCampaignFixture):
    """A restart gates all motion on an explicit operator acknowledgment."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_restart_recovery"
    LANE_TARGET = 1

    def _acknowledge(self):
        client = self.node.create_client(SetParameters, f"/{NODE_NAME}/set_parameters")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), "campaign parameter service")
        request = SetParameters.Request()
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        request.parameters = [ParameterMessage(name="restart_acknowledged", value=value)]
        future = client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done())
        self.assertTrue(future.result().results[0].successful)

    def test_restart_waits_for_acknowledgment_then_unproven_restock_stays_unresolved(self):
        self.spin_until(
            lambda: any(m.phase == Status.PHASE_RECOVERING for _, m in self.snapshot_statuses()),
            30.0,
            "PHASE_RECOVERING after the restart",
        )
        recovering = next(
            m for _, m in self.snapshot_statuses() if m.phase == Status.PHASE_RECOVERING
        )
        self.assertTrue(recovering.restart_recovery_pending)
        self.assertTrue(support.unresolved(recovering))
        self.assertIn("restart_acknowledged", recovering.detail)
        # Fresh work and elapsed time never resume motion: hold for several status periods.
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            self._publish_ready()
            time.sleep(0.05)
        self.assertEqual(self.sends(), [], "no goal is sent before the operator acknowledgment")
        self.assertTrue(
            all(
                m.phase in {Status.PHASE_WAITING_FOR_COORDINATOR, Status.PHASE_RECOVERING}
                for _, m in self.snapshot_statuses()
            ),
            "the campaign reports only waiting/recovering until acknowledged",
        )

        # The operator acknowledges; the tray is empty so the survey runs, then a valid
        # candidate lets the restock goal go out and fail without stop evidence.
        type(self).tray_script = ["no_candidate"]
        type(self).restock_script = ["execution_failed"]
        with self.world.lock:
            self.world.add_back("sim:restart_candidate", 1, "SIM-CAN-STD")
        ack_time = time.monotonic()
        self._acknowledge()
        self.spin_until(
            lambda: any(e[1] == "restock" and e[0] > ack_time for e in self.snapshot_events()),
            40.0,
            "the restock goal after the acknowledgment",
        )
        self.spin_until(
            lambda: any(
                support.unresolved(m) and not m.restart_recovery_pending and support.attempts(m)
                for t, m in self.snapshot_statuses()
                if t > ack_time
            ),
            15.0,
            "the unproven restock terminal to leave an unresolved attempt",
        )
        marked = next(
            m
            for t, m in self.snapshot_statuses()
            if t > ack_time and support.unresolved(m) and support.attempts(m)
        )
        self.assertTrue(any(entry.startswith("restock#") for entry in support.attempts(marked)))
        mark = len(self.sends())
        with self.world.lock:
            self.world.lanes["lane_03"]["invalid"] = True
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            self._publish_ready()
            time.sleep(0.05)
        self.assertEqual(
            len(self.sends()),
            mark,
            "no goal is sent after an unproven restock terminal, whatever the world offers",
        )
        self.assertTrue(rclpy.ok())
