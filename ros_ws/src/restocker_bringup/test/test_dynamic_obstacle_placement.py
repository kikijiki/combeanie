# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
The obstacle placement rule must never place the box on the arm's next start state.

The runtime test spawns an obstacle onto a configuration its own transfer recorded, and the next
transfer plans every grasp candidate from the state the arm still holds. When the recorded
sample the rule picks sits on that state, MoveIt's CheckStartStateCollision adapter refuses every
candidate before the planner runs — one generic FAILURE 99999 each — and recovery has nothing
left to spend: the refusal Card 010's SC-003 runs 6 and 7 recorded (Card 043).

Card 059 sharpened the question: the rule must clear the parked configuration under **full
state validity** (every link, the assertion-4 `/check_state_validity` answer), not a tool0-only
distance. The fixtures:

* ``obstacle_placement_recorded.json`` — one run's full corridor sample set (work/cmbompl-043,
  2026-09-26, deduplicated at 1 cm), with rail-clear samples far from the parked state; the
  rule must keep placing the obstacle there.
* ``obstacle_placement_no_rail_beating_sample.json`` — the same recording restricted to samples
  whose rail clearance does not exceed the parked state's own: the condition SC-003 runs 6/7
  must have had (their chosen box sat on the parked state itself).
* ``obstacle_placement_red_centres.json`` — the three obstacle centres Card 010's independent
  review receipted as red at 837817e (each clearing tool0 by 0.32–0.48 m, each colliding with
  upper_arm_link/shoulder_link at assertion 4) plus the green far-side centre from 7972246.
  The full-state rule must refuse every red centre the tool0 proxy would have accepted.
"""

import json
import os
from pathlib import Path

import pytest
import test_dynamic_obstacle_runtime as runtime

DATA = Path(os.environ["RESTOCKER_BRINGUP_SOURCE_DIR"]) / "test" / "data"
# Kept numerically identical to the runtime rule (PROJECTOR_PADDING_M, START_STATE_MARGIN_M)
# rather than imported, so this test runs against the pre-Card-043 rule too: the invariant is
# what it checks, not the implementation's own constants.
PROJECTOR_PADDING_M = 0.03
START_STATE_MARGIN_M = 0.15


def _load(name):
    payload = json.loads((DATA / name).read_text(encoding="utf-8"))
    samples = [(None, None, tuple(tool)) for tool in payload["samples"]]
    return payload, samples


def _load_red_centres():
    return json.loads((DATA / "obstacle_placement_red_centres.json").read_text(encoding="utf-8"))


def _clearance(sample_tool, parked_tool):
    half = [0.5 * size + PROJECTOR_PADDING_M for size in runtime.OBSTACLE_SIZE]
    return max(abs(sample_tool[axis] - parked_tool[axis]) - half[axis] for axis in range(3))


def _recorded_state_valid(payload):
    """
    Offline stand-in for the runtime predicate: MoveIt's recorded verdicts.

    False exactly on the centres the red runs' assertion 4 refused (contacts receipted in the
    fixture); True otherwise. This is what the full-state oracle knew from the receipts without
    a simulator; the live rule asks /check_state_validity for the same answer.
    """
    invalid = [tuple(entry["centre"]) for entry in payload["invalid_centres"]]

    def predicate(centre):
        return not any(
            all(abs(a - b) <= 0.0015 for a, b in zip(centre, bad, strict=True)) for bad in invalid
        )

    return predicate


def test_a_recording_with_a_rail_beating_sample_keeps_its_placement():
    """A good recording places the obstacle on a far, rail-clear configuration — unchanged."""
    payload, samples = _load("obstacle_placement_recorded.json")
    parked = samples[-1][2]
    # Full-state stub True: this fixture's far placements were live-verified valid at Card
    # 043's green gate; the red-centre tests below exercise the predicate itself.
    chosen = runtime.corridor_sample(samples, lambda centre: True)
    clearance = _clearance(chosen[2], parked)
    assert clearance >= START_STATE_MARGIN_M, (
        f"chosen tool {chosen[2]} sits {clearance:.4f} m from the padded box around the parked "
        f"tool {parked}"
    )
    rail = abs(chosen[2][0] - payload["obstructed_lane_center_x_m"])
    assert rail >= payload["obstacle_rail_clearance_m"]


def test_a_recording_without_a_rail_beating_sample_never_places_the_box_on_the_arm():
    """
    The SC-003 runs 6/7 condition: the box must still clear the next transfer's start state.

    On this set the pre-Card-043 rule returns the parked pose itself (rail clearance 0.4021 m,
    padded clearance -0.105 m — the box *contains* the parked tool), which is the refusal those
    runs recorded: every candidate refused by the same start-state collision.
    """
    payload, samples = _load("obstacle_placement_no_rail_beating_sample.json")
    parked = samples[-1][2]
    chosen = runtime.corridor_sample(samples, lambda centre: True)
    clearance = _clearance(chosen[2], parked)
    assert clearance >= START_STATE_MARGIN_M, (
        "the placement rule put the obstacle on the arm's own start state for the next "
        f"transfer: chosen tool {chosen[2]} against parked tool {parked}, padded clearance "
        f"{clearance:.4f} m (negative = inside the box); every grasp candidate plans from "
        "that state, so MoveIt refuses them all before the planner runs"
    )
    # The chosen sample must be one the recording actually contains, never a synthetic pose.
    assert chosen[2] in {sample[2] for sample in samples}


def test_a_recording_with_no_clear_configuration_says_so():
    """No clear configuration at all is a ValueError, not a placement that hides the problem."""
    park = (-1.0, -0.03, 1.44)
    samples = [
        (None, None, park),
        (None, None, (-1.005, -0.03, 1.42)),
        (None, None, (-0.99, -0.02, 1.45)),
    ]
    with pytest.raises(ValueError, match="own start state"):
        runtime.corridor_sample(samples, lambda centre: True)


def test_red_receipt_centres_pass_the_tool0_proxy_the_rule_used_to_trust():
    """
    The gap, documented: every red centre clears the tool0 margin the old rule trusted.

    These are the exact centres Card 010's review receipted (0.3175 / 0.3419 / 0.4822 m of
    tool0 clearance, assertion-4 contacts upper_arm_link / shoulder_link). Under the tool0-only
    prefilter they all pass; the full-state predicate below is what must stop them.
    """
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    predicate = _recorded_state_valid(payload)
    for entry in payload["invalid_centres"]:
        centre = tuple(entry["centre"])
        clearance = _clearance(centre, parked)
        assert clearance >= START_STATE_MARGIN_M, (
            f"red centre {centre} ({entry['run']}) no longer clears the tool0 proxy at "
            f"{clearance:.4f} m — fixture drifted?"
        )
        assert predicate(centre) is False, (
            f"red centre {centre} ({entry['run']}) must carry the recorded invalid verdict "
            f"(contacts {entry['contacts']})"
        )


def test_red_receipt_centres_are_what_the_tool0_only_rule_would_have_chosen():
    """
    On a red-run-shaped recording the max-rail tool0-only rule returns the owner-gate centre.

    That is exactly the placement that went red 3/3 at 837817e (no far-side samples in the
    recording, so the best tool0-clearing sample sits under the parked arm's links). This case
    re-derives the old rule (max rail key over tool0-clearing centres) rather than running the
    pre-Card-059 code, so it documents the gap; the cases below test the new rule.
    """
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    reds = [tuple(entry["centre"]) for entry in payload["invalid_centres"]]
    best = max(reds, key=lambda centre: abs(centre[0] - runtime.OBSTRUCTED_LANE_CENTER_X_M))
    assert best == (-0.998, 0.048, 0.843), (
        f"the tool0-only max-rail rule would pick {best}, not the receipted owner-gate centre"
    )
    for centre in reds:
        assert _clearance(centre, parked) >= START_STATE_MARGIN_M


def test_full_state_rule_refuses_a_red_only_recording_rather_than_placing():
    """A recording whose only corridor candidates are red centres raises — never places."""
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    samples = [(None, None, tuple(entry["centre"])) for entry in payload["invalid_centres"]]
    samples.append((None, None, parked))
    with pytest.raises(ValueError, match="full state validity"):
        runtime.corridor_sample(samples, _recorded_state_valid(payload))


def test_full_state_rule_places_on_the_green_far_side_sample_when_recorded():
    """When the recording carries the green far-side centre, the rule still chooses it."""
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    green = tuple(payload["valid_centres"][0]["centre"])
    samples = [(None, None, tuple(entry["centre"])) for entry in payload["invalid_centres"]]
    samples.append((None, None, green))
    samples.append((None, None, parked))
    chosen = runtime.corridor_sample(samples, _recorded_state_valid(payload))
    assert chosen[2] == green, (
        f"expected the green far-side centre {green}, got {chosen[2]} — a red centre won"
    )


# A synthetic full-state-valid corridor centre ranked BELOW every red centre on the rail (key
# 0.15 m vs about 0.40 m for the owner-gate red), clearing the tool0 prefilter. Not a receipt:
# it exists so the rule has to skip a higher-ranked refusal to reach it.
LOWER_RANKED_VALID = (-0.75, -0.2, 0.9)
# 2 cm from the owner-gate red centre, ranked between it and LOWER_RANKED_VALID: inside
# STATE_VALIDITY_DEDUP_M of an already-refused box, so it must share that verdict unasked.
NEAR_OWNER_GATE_RED = (-0.978, 0.048, 0.843)


def _counting(predicate):
    asked = []

    def counted(centre):
        asked.append(centre)
        return predicate(centre)

    return counted, asked


def test_full_state_rule_skips_a_higher_ranked_refusal_for_a_lower_ranked_valid_centre(capsys):
    """A red centre outranks a valid one on the rail: the rule refuses it, takes the valid one."""
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    owner_gate_red = tuple(payload["invalid_centres"][0]["centre"])
    for centre in (owner_gate_red, LOWER_RANKED_VALID):
        assert _clearance(centre, parked) >= START_STATE_MARGIN_M
    samples = [
        (None, None, owner_gate_red),
        (None, None, LOWER_RANKED_VALID),
        (None, None, parked),
    ]
    predicate, asked = _counting(_recorded_state_valid(payload))
    chosen = runtime.corridor_sample(samples, predicate)
    assert chosen[2] == LOWER_RANKED_VALID
    assert asked == [owner_gate_red, LOWER_RANKED_VALID]
    assert "2 distinct candidate box(es) judged against the parked configuration (1 refused)" in (
        capsys.readouterr().out
    )


def test_full_state_rule_does_not_re_ask_a_centre_within_the_dedup_distance_of_a_refusal(capsys):
    """A neighbour within STATE_VALIDITY_DEDUP_M of a refused box shares its verdict unasked."""
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    owner_gate_red = tuple(payload["invalid_centres"][0]["centre"])
    samples = [
        (None, None, owner_gate_red),
        (None, None, NEAR_OWNER_GATE_RED),
        (None, None, LOWER_RANKED_VALID),
        (None, None, parked),
    ]
    predicate, asked = _counting(_recorded_state_valid(payload))
    chosen = runtime.corridor_sample(samples, predicate)
    assert NEAR_OWNER_GATE_RED not in asked
    assert chosen[2] == LOWER_RANKED_VALID
    assert "2 distinct candidate box(es) judged against the parked configuration (1 refused)" in (
        capsys.readouterr().out
    )


def test_full_state_rule_fails_closed_with_a_cap_message_when_the_check_budget_runs_out(
    monkeypatch,
):
    """With the cap at 1 and a refused top candidate, the rule raises naming the cap."""
    payload = _load_red_centres()
    parked = tuple(payload["parked_tool"])
    owner_gate_red = tuple(payload["invalid_centres"][0]["centre"])
    samples = [
        (None, None, owner_gate_red),
        (None, None, LOWER_RANKED_VALID),
        (None, None, parked),
    ]
    monkeypatch.setattr(runtime, "MAX_STATE_VALIDITY_CHECKS", 1)
    predicate, asked = _counting(_recorded_state_valid(payload))
    with pytest.raises(ValueError, match="cap was reached with unjudged candidates left"):
        runtime.corridor_sample(samples, predicate)
    assert asked == [owner_gate_red]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
