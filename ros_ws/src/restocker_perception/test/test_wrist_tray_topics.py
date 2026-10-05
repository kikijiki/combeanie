# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Pin the tray duties' topic split: overview evidence can never reach the world state."""

# The world state ingests exactly one object topic and pins its single publisher. If the tray
# overview duty published there, overview-range hypotheses would be admitted as world objects and
# a transfer could be planned from evidence the specification reserves for CONFIRM. The split is
# configuration, so configuration is what must be pinned.

import os
from pathlib import Path

import yaml

WORLD_STATE_INGEST_TOPIC = "/perception/object_observations"
OVERVIEW_TOPIC = "/perception/tray_candidates"


def _tray_config() -> dict:
    path = Path(os.environ["RESTOCKER_TEST_TRAY_PERCEPTION_CONFIG"])
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _parameters(node: str) -> dict:
    return _tray_config()[node]["ros__parameters"]


def test_overview_candidates_never_share_the_world_state_ingest_topic() -> None:
    overview = _parameters("tray_overview_perception")
    confirm = _parameters("tray_confirm_perception")
    assert overview["observation_topic"] == OVERVIEW_TOPIC
    assert overview["observation_topic"] != WORLD_STATE_INGEST_TOPIC
    # The confirm duty is the one that publishes the replacing observation, on the admitted
    # topic, and is therefore the sole producer the world state's publisher pin may latch.
    assert confirm["observation_topic"] == WORLD_STATE_INGEST_TOPIC


def test_candidate_and_detection_topics_are_distinct_per_duty() -> None:
    overview = _parameters("tray_overview_perception")
    confirm = _parameters("tray_confirm_perception")
    assert overview["detection_topic"] == "/perception/tray_overview_detections"
    assert confirm["detection_topic"] == "/perception/tray_confirm_detections"
    assert overview["detection_topic"] != confirm["detection_topic"]
    # The duties share the wrist streams; only their outputs differ.
    for key in ("colour_topic", "depth_topic", "camera_info_topic"):
        assert overview[key] == confirm[key]


# Milestone 10 §7 / Cards 035+036: the confirm duty's claimed sigma floors the attachment
# fidelity budget (k * sigma + mechanical_margin, k = 3, margin = 1 mm, capped by the jaw-travel
# ceiling ~16 mm). The floor must cover this duty's measured accuracy against ground truth —
# not only top-face point scatter. Card 035 calibrated it to 0.004 m while the estimator still
# carried the ~5.5 mm world-Y bias (Card 004 confirm error 2.2-7.5 mm, attach residuals to
# 9.872 mm). Card 036 removed the bias (hull circle fit) and the predeclared confirm re-run
# measured norms 0.191-0.363 mm with |Y| <= 0.268 mm (n=12 admitted), so the floor returns to
# 0.002 m: 3*2+1 = 7 mm covers the post-fix distribution, stays under the ceiling, and the
# >= 13.5 mm evaluator single-frame tail still refuses. FIDELITY_SIGMA_MULTIPLIER /
# FIDELITY_MECHANICAL_MARGIN_M from attachment_boundary.yaml (the same constants
# restocker_benchmarks pins).
FIDELITY_SIGMA_MULTIPLIER = 3.0
FIDELITY_MECHANICAL_MARGIN_M = 0.001
FIDELITY_JAW_TRAVEL_CEILING_M = 0.016
# Post-fix Card 004-cell re-run (predeclared-2026-09-25-wrist-confirm-y-bias): worst admitted
# confirm residual norm. The floor's budget must cover it. The pre-fix 9.872 mm attach residual
# is the biased estimator's body and is deliberately not the comparator for the restored floor.
MEASURED_CONFIRM_ERROR_COVER_M = 0.000363


def test_confirm_sigma_floor_covers_measured_accuracy_inside_the_ceiling() -> None:
    confirm = _parameters("tray_confirm_perception")
    floor = confirm["translation_sigma_floor_m"]
    budget = min(
        FIDELITY_SIGMA_MULTIPLIER * floor + FIDELITY_MECHANICAL_MARGIN_M,
        FIDELITY_JAW_TRAVEL_CEILING_M,
    )
    # The claim covers the post-fix measured distribution (0.363 mm worst).
    assert budget >= MEASURED_CONFIRM_ERROR_COVER_M
    # Fail-closed: the budget is still a real bound under the mechanical ceiling, not a
    # ceiling-sized rubber stamp.
    assert budget < FIDELITY_JAW_TRAVEL_CEILING_M
    assert floor == 0.002
    # Overview candidates never authorize a grasp, so their floor is not held to this rule;
    # it must at least remain a positive accuracy claim.
    overview = _parameters("tray_overview_perception")
    assert overview["translation_sigma_floor_m"] > 0.0


def test_evaluator_reads_overview_candidates_from_the_candidates_topic() -> None:
    evaluator = _parameters("tray_pose_error_evaluator")
    assert evaluator["candidate_topic"] == OVERVIEW_TOPIC
    assert evaluator["estimate_topic"] == WORLD_STATE_INGEST_TOPIC
    assert evaluator["ground_truth_topic"] != evaluator["estimate_topic"]


def test_both_duties_keep_the_backend_names_the_survey_filters_on() -> None:
    # tray_survey_node selects the right stream by backend name; renaming a duty here without
    # renaming the node's default would silently starve the survey of observations.
    assert _parameters("tray_overview_perception")["backend_name"] == "wrist_rgbd_tray_overview"
    assert _parameters("tray_confirm_perception")["backend_name"] == "wrist_rgbd_tray_confirm"
