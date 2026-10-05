# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Wait until the simulation clock is publishing before any controller is activated."""

# The controller manager runs its update loop from process start and, until a /clock message
# arrives, uses the time argument it was handed ("No clock received, using time argument
# instead"). The first update after the clock arrives measures a period across that
# discontinuity, which can be negative. A joint_trajectory_controller commanding velocity runs a
# PID, control_toolbox throws std::invalid_argument on a negative dt, and the controller manager
# deactivates the controller that threw — leaving one the spawner already reported as activated
# unable to accept a trajectory. Waiting for the clock bridge before any controller is activated
# keeps the manager's time base monotonic across the switch.
#
# One message here proves the publisher is discoverable, not that the controller manager's own
# subscription has matched, so a short settle window runs after the first message.
#
# The discovery bound is 180 s, not 60 s: under full-suite parallel load the first wave of
# simulator compositions delayed the first /clock past 60 s and the wait shut the launch down
# (test_lane_survey_runtime exited 1 with "no /clock message arrived before the controllers were
# spawned"). `ros2 topic echo --once` is unusable for this job: it holds its discovery window
# open for the whole spin time even after the message arrives, adding that delay to every launch.

from __future__ import annotations

import time

DISCOVERY_TIMEOUT_S = 180.0
SETTLE_S = 1.0


def wait_for_clock(
    received: list,
    spin_once,
    *,
    timeout_s: float = DISCOVERY_TIMEOUT_S,
    monotonic=time.monotonic,
) -> bool:
    """
    Spin until ``received`` is non-empty, then settle; return whether the clock arrived.

    ``received`` is the live list a /clock subscription appends to, ``spin_once`` advances the
    executor once (taking a timeout in seconds), and ``monotonic`` is injectable so tests can
    drive the deadline without wall-clock waiting.
    """
    deadline = monotonic() + timeout_s
    while not received and monotonic() < deadline:
        spin_once(0.1)
    if not received:
        return False
    settle_until = monotonic() + SETTLE_S
    while monotonic() < settle_until:
        spin_once(0.1)
    return True


def main() -> int:
    """Exit 0 once /clock has published, 1 if it never arrived inside the discovery bound."""
    import rclpy
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("wait_for_simulation_clock")
    received: list = []
    node.create_subscription(Clock, "/clock", received.append, 10)
    ok = wait_for_clock(
        received, lambda timeout_sec: rclpy.spin_once(node, timeout_sec=timeout_sec)
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
