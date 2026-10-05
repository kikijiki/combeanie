# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Package gate for the public demo's autonomous-campaign wiring."""

import os
from pathlib import Path
import re


def verify_just_demo_launches_the_sensor_driven_campaign():
    repository = Path(os.environ["RESTOCKER_REPOSITORY_ROOT"])
    justfile = (repository / "justfile").read_text()
    demo = (repository / "scripts" / "demo.bash").read_text()
    baseline = (
        repository / "ros_ws" / "src" / "restocker_bringup" / "launch" / "baseline.launch.py"
    ).read_text()

    assert 'demo max_cycles="0" gt="false": build' in justfile
    assert "scripts/demo.bash" in justfile
    assert "autonomous_campaign:=true" in demo
    # Card 086 stage 1: the campaign gates all motion on a restart acknowledgment; the demo's
    # fresh simulated world is the one launch that may acknowledge it up front.
    assert "campaign_restart_acknowledged:=true" in demo
    assert "campaign_restart_acknowledged" in baseline
    assert '"restart_acknowledged"' in baseline
    assert "task_execution_timeout_ms:=180000" in demo
    # lane_policy_state points at a path that does not exist so a stale persisted SetLanePolicy
    # document cannot overlay the fixture's targets. Persistence stays enabled — a SetLanePolicy
    # write would create the policy file — but nothing overlays the declared targets unless such
    # a write creates the file.
    assert "demo_lane_policy_state.absent" in demo

    # The demo has two labelled modes and only two: the shipped sensor-driven default, which
    # carries no evidence pins at all, and the ground-truth opt-in, which carries every pin.
    assert 'if [[ "$ground_truth" == "true" ]]; then' in demo
    gated, after_else = demo.split('if [[ "$ground_truth" == "true" ]]; then', 1)[1].split(
        "\nelse\n", 1
    )
    default_path = after_else

    # Ground-truth opt-in: dense fixture, GT back on the world state's ingest topic, GT lane
    # evidence, wrist tray duties off, tight GT evidence ages.
    assert "dense_restock_products.yaml" in gated
    assert "dense_restock_lanes.yaml" in gated
    assert "object_observation_topic:=/perception/object_observations" in gated
    assert "lane_observation_topic:=/perception/ground_truth/lane_observations" in gated
    assert "tray_overview_perception:=false" in gated
    assert "tray_confirm_perception:=false" in gated
    assert "product_observation_max_age:=2.0" in gated
    assert "selection_object_max_age_ms:=2000" in gated
    # The opt-in must reproduce the pre-flip demo exactly, including the two evidence horizons
    # whose launch defaults the sensor-driven switch raised (60000 -> 180000, 2000 -> 180000).
    assert "lane_evidence_validity_ms:=60000" in gated
    assert "maximum_observation_age_ms:=2000" in gated
    assert "placement_require_column_growth:=false" in gated

    # Sensor-driven default: an empty pin list is the assertion — baseline.launch.py's own
    # defaults are the shipped configuration (ground truth demoted, wrist duties owning the
    # ingest topics, 180 s evidence ages, overhead camera obstacles-only) on the sensor
    # acceptance fixture: the dense scenario's camera identity works, but the dense scenario
    # has not yet completed a sensor-driven run.
    assert "evidence_args=()" in default_path
    assert "object_observation_topic:=" not in default_path
    assert "lane_observation_topic:=" not in default_path
    assert "tray_confirm_perception:=" not in default_path
    assert "sensor_acceptance_products.yaml" in default_path
    assert "sensor_acceptance_lanes.yaml" in default_path

    assert 'executable="autonomous_restock_campaign"' in baseline
    assert "condition=IfCondition(autonomous_campaign)" in baseline
    assert '"product_catalog_path": product_catalog' in baseline
    assert '"max_observation_age_sec": ParameterValue(' in baseline
    assert '"selection.maximum_object_age_ms": ParameterValue(' in baseline
    assert '"task.execution_timeout_ms": ParameterValue(' in baseline
    assert '"placement_require_column_growth": placement_require_column_growth' in baseline
    # The mode loop's SURVEY_TRAY path is the coordinated action, so the campaign must start
    # its server beside the survey servers it already composes.
    assert "tray_survey_active = OrSubstitution(tray_survey, autonomous_campaign)" in baseline
    assert "condition=IfCondition(tray_survey_active)" in baseline


def verify_campaign_ghost_gate_follows_the_coordinator_selection_horizon():
    """
    Card 069 (review cmbrev069b B1): one launch argument feeds both horizons.

    The coordinator may pick any tracked object younger than selection.maximum_object_age_ms
    on an unpinned transfer; the campaign's retired-ghost gate holds for
    max(tray_evidence_validity, coordinator_object_max_age_ms). Both must come from the same
    selection_object_max_age_ms argument so they can never drift apart.
    """
    repository = Path(os.environ["RESTOCKER_REPOSITORY_ROOT"])
    baseline = (
        repository / "ros_ws" / "src" / "restocker_bringup" / "launch" / "baseline.launch.py"
    ).read_text()

    def node_block(variable):
        start = baseline.index(f"    {variable} = Node(")
        end = baseline.index("\n    )\n", start)
        return baseline[start:end]

    def fed_by_selection_horizon(block, parameter):
        pattern = (
            rf'"{re.escape(parameter)}":\s*ParameterValue\(\s*'
            r"selection_object_max_age_ms,\s*value_type=int\s*\)"
        )
        return re.search(pattern, block) is not None

    assert 'LaunchConfiguration("selection_object_max_age_ms")' in baseline
    assert fed_by_selection_horizon(
        node_block("task_coordinator_node"), "selection.maximum_object_age_ms"
    )
    assert fed_by_selection_horizon(
        node_block("autonomous_campaign_node"), "coordinator_object_max_age_ms"
    )


if __name__ == "__main__":
    verify_just_demo_launches_the_sensor_driven_campaign()
    verify_campaign_ghost_gate_follows_the_coordinator_selection_horizon()
