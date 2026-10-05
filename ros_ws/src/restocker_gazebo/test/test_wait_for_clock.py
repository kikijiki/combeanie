# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The simulation-clock wait must survive a late first message under first-wave load."""

from restocker_gazebo.wait_for_clock import DISCOVERY_TIMEOUT_S, SETTLE_S, wait_for_clock


class FakeClock:
    """A monotonic clock the test advances only while the executor spins."""

    def __init__(self):
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _spinner(clock: FakeClock, received: list, first_message_at: float | None):
    """Return a spin_once that advances fake time and delivers /clock at ``first_message_at``."""

    def spin_once(timeout_sec: float) -> None:
        clock.now += timeout_sec
        if first_message_at is not None and clock.now >= first_message_at and not received:
            received.append(object())

    return spin_once


def test_a_message_after_the_old_sixty_second_bound_still_counts():
    """A first /clock at 70 s must succeed; the pre-fix 60 s bound shut the launch down."""
    clock = FakeClock()
    received: list = []
    assert wait_for_clock(received, _spinner(clock, received, 70.0), monotonic=clock)
    # The settle window runs only after the message arrives.
    assert clock.now >= 70.0 + SETTLE_S


def test_no_message_fails_exactly_at_the_discovery_bound():
    """Silence must fail — and only at the shipped bound, not earlier."""
    clock = FakeClock()
    received: list = []
    assert not wait_for_clock(received, _spinner(clock, received, None), monotonic=clock)
    assert clock.now >= DISCOVERY_TIMEOUT_S


def test_shipped_discovery_bound_covers_first_wave_startup():
    """Pin the bound that closed the run-3-style failure; 60 s was measured too tight."""
    assert DISCOVERY_TIMEOUT_S >= 180.0


if __name__ == "__main__":
    test_a_message_after_the_old_sixty_second_bound_still_counts()
    test_no_message_fails_exactly_at_the_discovery_bound()
    test_shipped_discovery_bound_covers_first_wave_startup()
