# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Group terminal restock outcomes by cause."""
# The unit of a benchmark result is the distinct cause with its frequency, not a pass count.
#
# Two grouping keys are produced for every outcome and both are reported:
#
#   signature  the coordinator's terminal ``detail`` with every number masked. Derived from the
#              message the system emitted, so two runs share it only if the system said the same
#              thing about them.
#   family     a coarse label from the table below, for reading a campaign at a glance.
#
# The family never replaces the signature: an unrecognised detail is labelled ``unclassified`` and
# still carries its full signature into the report, so a new failure mode shows up as a new row.

from __future__ import annotations

import re

# Mirrors restocker_interfaces/action/RestockProduct. Restated rather than imported so a stored
# campaign can be re-aggregated without a ROS environment; the numbers are a published wire
# contract.
STATUS_NAMES: dict[int, str] = {
    0: "UNSET",
    1: "SUCCEEDED",
    2: "CANCELED",
    3: "NO_COMPATIBLE_PAIR",
    4: "VALIDATION_FAILED",
    5: "PLANNING_FAILED",
    6: "EXECUTION_FAILED",
    7: "VERIFICATION_FAILED",
    8: "RECOVERY_EXHAUSTED",
    9: "EXTERNAL_INCONSISTENCY",
    10: "OPERATOR_REQUIRED",
    11: "SHUTDOWN",
    13: "SKIPPED_RECOVERABLE",
    255: "INTERNAL_ERROR",
}

STATUS_SUCCEEDED = 1

# Outcomes the harness itself observed, not ones the coordinator reported. Kept separate because
# "the launch died" and "the coordinator refused the placement" are different claims.
HARNESS_STATUS = -1

# Short labels for the outcomes the harness itself reports; the full reason stays in the
# signature. Both the short reason the runner records and the sentence it writes into the detail
# are listed, so a record written before the short reason existed still gets the short label.
HARNESS_FAMILIES: dict[str, str] = {
    "goal was rejected": "goal_rejected",
    "the coordinator refused to accept the goal": "goal_rejected",
    "goal was never answered": "goal_unanswered",
    "send_goal_async did not complete": "goal_unanswered",
    "no result before the harness deadline": "no_result",
    "scenario product was never observed": "product_unobserved",
    "": "unknown",
}

_NUMBER = re.compile(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")
_WHITESPACE = re.compile(r"\s+")

# Ordered: the first family whose markers all appear wins. Order matters where a detail mentions
# more than one subsystem: "attachment submission was refused" names both the attachment and the
# refusal, and the attachment is the more specific claim.
_FAMILY_RULES: tuple[tuple[str, tuple[str, ...]], ...] = (
    # Milestone 10 §6 rung 5 (Card 051): the typed recoverable skip is an outcome, not a
    # failure — first so no generic rule (recovery, timeout, cancel) can absorb it.
    ("recoverable_skip", ("recoverable skip",)),
    ("cartesian_truncated", ("linear path",)),
    # The controller's own verdict, which MoveItMotionPort carries into the detail. These come
    # before the MoveIt rules because MoveIt reports every controller failure as CONTROL_FAILED;
    # matching "moveit reported" first would merge a path-tolerance abort, a rejected goal and a
    # lost controller connection into one row.
    #
    # The impulse row is claimed only when the port observed a joint of the executed trajectory at
    # its URDF velocity limit (the Signature B fingerprint), a speed no plan on this robot
    # commands. A path-tolerance abort with no such joint stays in the generic row.
    (
        "controller_velocity_limit_impulse",
        ("aborted with path_tolerance_violated", "reached its urdf velocity limit of"),
    ),
    ("controller_path_tolerance", ("aborted with path_tolerance_violated",)),
    ("controller_goal_tolerance", ("aborted with goal_tolerance_violated",)),
    ("controller_abort", ("' aborted, reporting ",)),
    ("controller_abort", ("' aborted with ",)),
    # A CONTROL_FAILED whose controller line never arrived: the reason was not established, but
    # establishing it was attempted, so it is not generic execution failure.
    ("controller_abort_unreported", ("no controller abort was reported",)),
    ("moveit_planning", ("moveit reported", "plan")),
    ("moveit_execution", ("moveit reported", "execut")),
    ("moveit_execution", ("path_tolerance_violated",)),
    ("moveit_execution", ("goal_tolerance_violated",)),
    ("moveit_execution", ("control_failed",)),
    ("moveit_planning", ("no_ik_solution",)),
    ("moveit_planning", ("goal_in_collision",)),
    ("moveit_planning", ("start_state_in_collision",)),
    ("moveit_planning", ("pose target was rejected",)),
    # The arm may already have been moving when MoveIt threw, so this is execution, not planning,
    # even though the phrase names both.
    ("moveit_execution", ("plan-and-execute",)),
    ("moveit_unavailable", ("movegroup is unavailable",)),
    ("moveit_unavailable", ("never published a current robot state",)),
    ("attachment", ("attachment",)),
    ("attachment", ("detachment",)),
    ("gripper", ("gripper",)),
    ("gripper", ("jaw",)),
    ("placement_verification", ("placement",)),
    ("grasp_verification", ("grasp", "verif")),
    ("grasp_generation", ("grasp",)),
    ("planning_scene", ("planning-scene",)),
    ("planning_scene", ("projector",)),
    ("selection", ("compatible",)),
    ("selection", ("selection",)),
    ("selection", ("no admissible",)),
    ("world_state", ("world state",)),
    ("world_state", ("reservation",)),
    ("world_state", ("snapshot",)),
    ("recovery", ("recovery",)),
    ("timeout", ("deadline",)),
    ("timeout", ("timed out",)),
    ("timeout", ("timeout",)),
    ("cancellation", ("cancel",)),
    ("shutdown", ("shutdown",)),
    # Observed: "motion inhibited; operator required" after a detach the coordinator could not
    # reconcile. Inhibition is its own outcome, not a motion failure: the system stopped itself and
    # refuses every later goal in the run until a human intervenes.
    ("inhibited", ("inhibit",)),
    ("operator_required", ("operator required",)),
    ("coordinator_internal", ("inbox",)),
    ("coordinator_internal", ("coordinator",)),
    # Last and deliberately vague: a detail that names motion but matched none of the specific
    # rules above. The signature still separates the individual causes inside it.
    ("motion_other", ("motion",)),
)


def normalize_detail(detail: str) -> str:
    """Mask the varying parts of a terminal detail so equal causes compare equal."""
    # Numbers are what varies between occurrences of the same cause: the Cartesian truncation
    # detail carries a percentage and a waypoint count, the MoveIt reason a numeric code beside
    # its name, the timeout details durations. Masking keeps the name and the sentence.
    text = _NUMBER.sub("#", detail.strip().lower())
    text = _WHITESPACE.sub(" ", text)
    # Object and lane identifiers survive masking as bare "#": the cause is the same for lane_01
    # and lane_04.
    return text


def classify_family(status: int, detail: str) -> str:
    """Return the coarse family for one terminal outcome."""
    if status == STATUS_SUCCEEDED:
        return "succeeded"
    text = detail.strip().lower()
    if status == HARNESS_STATUS:
        # The harness names its own outcomes, so nothing is inferred, only shortened. An unlisted
        # reason keeps its full text.
        return f"harness:{HARNESS_FAMILIES.get(text, text)}"
    for family, markers in _FAMILY_RULES:
        if all(marker in text for marker in markers):
            return family
    if not text:
        # A terminal status with no detail is itself a finding.
        return f"undetailed:{STATUS_NAMES.get(status, str(status)).lower()}"
    return "unclassified"


def signature(status: int, detail: str, cause: str = "") -> str:
    """Return the reported grouping key: the status name, the masked detail, and the cause."""
    name = "HARNESS" if status == HARNESS_STATUS else STATUS_NAMES.get(status, f"STATUS_{status}")
    normalized = normalize_detail(detail)
    key = f"{name}: {normalized}" if normalized else name
    normalized_cause = normalize_detail(cause)
    if normalized_cause and normalized_cause != normalized:
        # No pipe: a signature is a cell in a Markdown table in the generated report.
        key = f"{key} <- {normalized_cause}"
    return key


# The second axis: how far a goal got before it stopped. Cause and depth together separate "never
# planned the approach" from "grazed the divider on the insert". Mirrors the feedback constants in
# RestockProduct.action, restated for the same reason STATUS_NAMES is.
STATE_NAMES: dict[int, str] = {
    0: "IDLE",
    1: "VALIDATE_SCENE",
    2: "SELECT_PAIR",
    3: "RESERVE_TASK",
    4: "GENERATE_GRASPS",
    5: "PLAN_PRE_GRASP",
    6: "EXECUTE_PRE_GRASP",
    7: "PLAN_APPROACH",
    8: "EXECUTE_APPROACH",
    9: "CLOSE_GRIPPER",
    10: "VERIFY_GRASP",
    11: "ATTACH_TRANSACTION",
    12: "PLAN_RETRACT",
    13: "EXECUTE_RETRACT",
    14: "OBSERVE_DESTINATION",
    15: "GENERATE_PLACEMENT",
    16: "PLAN_PRE_INSERT",
    17: "EXECUTE_PRE_INSERT",
    18: "PLAN_INSERT",
    19: "EXECUTE_INSERT",
    20: "OPEN_GRIPPER",
    21: "DETACH_TRANSACTION",
    22: "VERIFY_PLACEMENT",
    23: "PLAN_RETREAT",
    24: "EXECUTE_RETREAT",
    25: "UPDATE_INVENTORY",
    26: "CANCEL_ACTIVE_MOTION",
    27: "RELEASE_TASK",
    28: "RECOVER",
    29: "FAULT",
    30: "REQUEST_OPERATOR",
    31: "COMPLETE",
    32: "CANCELED",
    33: "OPEN_GRIPPER_FOR_APPROACH",
}

# The nominal forward order of a complete transfer. "Furthest reached" is measured against this
# list, not the raw state number, because identifiers were assigned in the order the states were
# written: OPEN_GRIPPER_FOR_APPROACH is 33 but happens before CLOSE_GRIPPER (9).
NOMINAL_STATE_ORDER: tuple[int, ...] = (
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    33,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    27,
    31,
)

_NOMINAL_RANK = {state: rank for rank, state in enumerate(NOMINAL_STATE_ORDER)}


def furthest_state(states: list[int]) -> int | None:
    """Return the furthest nominal state in a feedback trace, ignoring off-path states."""
    # RECOVER, FAULT, CANCEL_ACTIVE_MOTION and REQUEST_OPERATOR are not on the forward path, so
    # they are excluded from depth; entering them is carried by the status and detail.
    ranked = [state for state in states if state in _NOMINAL_RANK]
    if not ranked:
        return None
    return max(ranked, key=lambda state: _NOMINAL_RANK[state])


def state_name(state: int | None) -> str:
    """Return a stable name for a task state, or ``none`` when no feedback arrived."""
    if state is None:
        return "none"
    return STATE_NAMES.get(state, f"STATE_{state}")


# States entered because something went wrong. The detail a goal publishes on entering one of
# these names the cause.
_OFF_PATH_STATES = (29, 30, 26, 28)  # FAULT, REQUEST_OPERATOR, CANCEL_ACTIVE_MOTION, RECOVER


def phase_durations(trace: list[dict], total_sim_s: float) -> dict[str, float]:
    """Split a feedback trace's simulation time into motion and everything else."""
    # Motion and planning are not split, because a motion port plans and executes a segment in one
    # call: the PLAN_* state holds for the whole plan-and-execute and the paired EXECUTE_* state
    # only records that the trajectory ran. Splitting them gave "planning 35 s, execution 0.0 s" on
    # every successful goal, an artefact of the state machine's shape.
    #
    # Feedback carries the sim time since the goal was accepted, so the time a state held is the
    # gap to the next transition; the last state closes against the result's total.
    motion = 0.0
    other = 0.0
    for index, entry in enumerate(trace):
        start = float(entry.get("elapsed_sim_s", 0.0))
        end = (
            float(trace[index + 1].get("elapsed_sim_s", 0.0))
            if index + 1 < len(trace)
            else total_sim_s
        )
        held = max(0.0, end - start)
        name = str(entry.get("state_name") or state_name(int(entry.get("state", -1))))
        if name.startswith(("PLAN_", "EXECUTE_")):
            motion += held
        else:
            other += held
    return {"motion_sim_s": motion, "other_sim_s": other}


def cause_detail(trace: list[dict]) -> str:
    """Return the fault-state feedback detail, which is often more specific than the result."""
    # Three runs of the baseline scenario terminated with the result detail "motion inhibited;
    # operator required", which names no cause. The FAULT feedback published a step earlier said
    # "attachment physically applied but not committed: simulated detach is applied but world
    # state refused the commit until its deadline". Grouping on the result detail alone would merge
    # every inhibition into one row.
    for entry in reversed(trace):
        if int(entry.get("state", -1)) in _OFF_PATH_STATES and entry.get("detail"):
            return str(entry["detail"])
    return ""
