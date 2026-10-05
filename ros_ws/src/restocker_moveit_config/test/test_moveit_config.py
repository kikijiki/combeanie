# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contract tests for MoveIt semantics and configuration."""

import importlib.util
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

SOURCE_DIR = Path(os.environ["RESTOCKER_MOVEIT_CONFIG_SOURCE_DIR"])
DESCRIPTION_SOURCE = Path(os.environ["RESTOCKER_DESCRIPTION_SOURCE_DIR"])
CONTROL_SOURCE = Path(os.environ["RESTOCKER_CONTROL_SOURCE_DIR"])

PLANNING_XACRO = DESCRIPTION_SOURCE / "urdf" / "restocker_planning.urdf.xacro"
SRDF_FILE = SOURCE_DIR / "config" / "restocker.srdf"
MOVING_JOINTS = {
    "rail_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "left_finger_joint",
    "right_finger_joint",
}
MANIPULATOR_JOINTS = {
    "rail_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
}
EXPECTED_DISABLED_PAIRS = {
    frozenset(pair)
    for pair in (
        ("rail_base", "carriage"),
        ("carriage", "base_link_inertia"),
        ("base_link", "base_link_inertia"),
        ("base_link_inertia", "shoulder_link"),
        ("shoulder_link", "upper_arm_link"),
        ("upper_arm_link", "forearm_link"),
        ("forearm_link", "wrist_1_link"),
        ("wrist_1_link", "wrist_2_link"),
        ("wrist_2_link", "wrist_3_link"),
        ("tool0", "wrist_1_link"),
        ("tool0", "wrist_2_link"),
        ("tool0", "wrist_3_link"),
        ("wrist_1_link", "wrist_3_link"),
        ("gripper", "left_finger"),
        ("gripper", "right_finger"),
    )
}


def _load_yaml(name: str) -> dict:
    return yaml.safe_load((SOURCE_DIR / "config" / name).read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def robot() -> ET.Element:
    expanded = subprocess.run(
        ["xacro", str(PLANNING_XACRO)], check=True, capture_output=True, text=True
    ).stdout
    return ET.fromstring(expanded)


@pytest.fixture(scope="module")
def semantics() -> ET.Element:
    return ET.parse(SRDF_FILE).getroot()


def test_setup_assistant_metadata_points_to_owned_sources() -> None:
    metadata = yaml.safe_load((SOURCE_DIR / ".setup_assistant").read_text(encoding="utf-8"))
    configuration = metadata["moveit_setup_assistant_config"]
    assert configuration["urdf"] == {
        "package": "restocker_description",
        "relative_path": "urdf/restocker_planning.urdf.xacro",
    }
    assert configuration["srdf"] == {"relative_path": "config/restocker.srdf"}


def test_srdf_groups_frames_and_named_states_match_urdf(
    robot: ET.Element, semantics: ET.Element
) -> None:
    assert semantics.attrib == {"name": "restocker"}
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    assert semantics.find("virtual_joint") is None
    child_links = {joint.find("child").attrib["link"] for joint in robot.findall("joint")}
    assert links - child_links == {"world"}
    assert robot.find("link[@name='world']").find("inertial") is None

    groups = {group.attrib["name"]: group for group in semantics.findall("group")}
    assert set(groups) == {"manipulator", "arm", "gripper"}
    assert groups["manipulator"].find("chain").attrib == {
        "base_link": "rail_base",
        "tip_link": "tool0",
    }
    assert groups["arm"].find("chain").attrib == {
        "base_link": "base_link",
        "tip_link": "tool0",
    }
    gripper_joints = {item.attrib["name"] for item in groups["gripper"].findall("joint")}
    assert gripper_joints == {"left_finger_joint", "right_finger_joint"}
    assert gripper_joints <= joints.keys()

    end_effector = semantics.find("end_effector")
    assert end_effector.attrib == {
        "name": "restocker_gripper",
        "parent_link": "tool0",
        "group": "gripper",
        "parent_group": "manipulator",
    }
    assert end_effector.attrib["parent_link"] in links

    states = {
        (state.attrib["group"], state.attrib["name"]): {
            joint.attrib["name"]: float(joint.attrib["value"]) for joint in state.findall("joint")
        }
        for state in semantics.findall("group_state")
    }
    assert set(states[("manipulator", "home")]) == MANIPULATOR_JOINTS
    assert set(states[("gripper", "open")]) == gripper_joints
    assert set(states[("gripper", "closed")]) == gripper_joints
    for state_name in ("open", "closed"):
        values = states[("gripper", state_name)]
        assert values["left_finger_joint"] == values["right_finger_joint"]


def test_collision_exclusions_match_the_upstream_ur10e_and_cell_mount(
    robot: ET.Element, semantics: ET.Element
) -> None:
    disabled = {
        frozenset((entry.attrib["link1"], entry.attrib["link2"]))
        for entry in semantics.findall("disable_collisions")
    }
    assert disabled == EXPECTED_DISABLED_PAIRS
    adjacent = {
        frozenset((joint.find("parent").attrib["link"], joint.find("child").attrib["link"]))
        for joint in robot.findall("joint")
    }
    never = {
        frozenset((entry.attrib["link1"], entry.attrib["link2"]))
        for entry in semantics.findall("disable_collisions")
        if entry.attrib["reason"] == "Never"
    }
    assert disabled - never <= adjacent
    assert never == {
        frozenset(("carriage", "base_link_inertia")),
        frozenset(("tool0", "wrist_1_link")),
        frozenset(("tool0", "wrist_2_link")),
        frozenset(("tool0", "wrist_3_link")),
        frozenset(("wrist_1_link", "wrist_3_link")),
    }


def test_kinematics_uses_an_installed_kdl_plugin() -> None:
    kinematics = _load_yaml("kinematics.yaml")
    assert set(kinematics) == {"manipulator", "arm"}
    for configuration in kinematics.values():
        assert configuration == {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_search_resolution": pytest.approx(0.005),
            "kinematics_solver_timeout": pytest.approx(0.05),
        }

    share = Path(get_package_share_directory("moveit_kinematics"))
    plugin_descriptions = "\n".join(
        path.read_text(encoding="utf-8") for path in share.rglob("*.xml")
    )
    assert "kdl_kinematics_plugin/KDLKinematicsPlugin" in plugin_descriptions


def test_moveit_limits_cover_all_moving_joints_and_never_exceed_urdf(
    robot: ET.Element,
) -> None:
    configured = _load_yaml("joint_limits.yaml")["joint_limits"]
    assert set(configured) == MOVING_JOINTS
    urdf_joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    for name, limits in configured.items():
        assert limits["has_velocity_limits"] is True
        assert (
            0.0
            < limits["max_velocity"]
            <= float(urdf_joints[name].find("limit").attrib["velocity"])
        )
        assert limits["has_acceleration_limits"] is True
        assert limits["max_acceleration"] > 0.0


def test_moveit_position_limits_stay_clear_of_the_enforced_urdf_bounds(
    robot: ET.Element,
) -> None:
    """Require a margin between what MoveIt plans and what controller_manager enforces."""
    # joint_limits::compute_velocity_limits zeroes a velocity command once a joint is more than
    # 0.002 rad outside a bound, with no direction check, so a joint nudged past its limit cannot
    # be driven back. Planning onto the bound is one rounding error from an unrecoverable stall;
    # this asserts the margin that keeps plans clear of it.
    # The arm and rail carry 0.02; the jaws travel their whole short stroke by design and carry
    # 0.001, which is still half the 0.002 rad threshold that strands a joint.
    minimum_margin = 0.001
    configured = _load_yaml("joint_limits.yaml")["joint_limits"]
    urdf_joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    for name, limits in configured.items():
        assert limits["has_position_limits"] is True, name
        urdf_limit = urdf_joints[name].find("limit")
        urdf_lower = float(urdf_limit.attrib["lower"])
        urdf_upper = float(urdf_limit.attrib["upper"])
        assert limits["min_position"] >= urdf_lower + minimum_margin, name
        assert limits["max_position"] <= urdf_upper - minimum_margin, name
        assert limits["min_position"] < limits["max_position"], name


def test_ompl_pipeline_is_single_bounded_and_explicit() -> None:
    ompl = _load_yaml("ompl_planning.yaml")
    assert ompl["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert ompl["planner_configs"] == {
        "RRTConnectkConfigDefault": {"type": "geometric::RRTConnect", "range": 0.0}
    }
    assert set(ompl["manipulator"]["planner_configs"]) == {"RRTConnectkConfigDefault"}
    assert set(ompl["arm"]["planner_configs"]) == {"RRTConnectkConfigDefault"}
    # Projection onto the transfer-box manifold created exact-face paths that became invalid after
    # time parameterization. Joint-space rejection keeps samples in the volume; the motion port
    # still checks the final trajectory against its separate outer box.
    assert ompl["manipulator"]["enforce_joint_model_state_space"] is True
    assert "enforce_joint_model_state_space" not in ompl["arm"]
    assert ompl["request_adapters"] == [
        "default_planning_request_adapters/ResolveConstraintFrames",
        "default_planning_request_adapters/ValidateWorkspaceBounds",
        "default_planning_request_adapters/CheckStartStateBounds",
        "default_planning_request_adapters/CheckStartStateCollision",
    ]
    assert ompl["response_adapters"] == [
        "default_planning_response_adapters/AddTimeOptimalParameterization",
        "default_planning_response_adapters/ValidateSolution",
        "default_planning_response_adapters/DisplayMotionPath",
    ]
    assert 0.0 < ompl["start_state_max_bounds_error"] <= 0.05
    assert ompl["totg"] == {
        "path_tolerance": pytest.approx(0.001),
        "resample_dt": pytest.approx(0.1),
        "min_angle_change": pytest.approx(0.001),
    }


def test_moveit_controller_mapping_matches_ros2_control_partition() -> None:
    moveit = _load_yaml("moveit_controllers.yaml")
    control = yaml.safe_load(
        (CONTROL_SOURCE / "config" / "controllers.yaml").read_text(encoding="utf-8")
    )
    manager = moveit["moveit_simple_controller_manager"]
    expected_names = {"arm_controller", "rail_controller", "gripper_controller"}
    assert set(manager["controller_names"]) == expected_names

    mapped_joints = []
    for name in expected_names:
        mapping = manager[name]
        assert mapping["type"] == "FollowJointTrajectory"
        assert mapping["action_ns"] == "follow_joint_trajectory"
        assert mapping["default"] is True
        assert mapping["joints"] == control[name]["ros__parameters"]["joints"]
        mapped_joints.extend(mapping["joints"])
    assert set(mapped_joints) == MOVING_JOINTS
    assert len(mapped_joints) == len(set(mapped_joints))

    execution = moveit["trajectory_execution"]
    assert execution["execution_duration_monitoring"] is True
    assert execution["allowed_start_tolerance"] <= 0.01


def test_moveit_does_not_publish_planning_urdf_on_hardware_topic() -> None:
    launch_path = SOURCE_DIR / "launch" / "move_group.launch.py"
    module_spec = importlib.util.spec_from_file_location(
        "restocker_move_group_launch", launch_path
    )
    assert module_spec is not None
    assert module_spec.loader is not None
    launch_module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(launch_module)

    monitor = launch_module._moveit_configuration().planning_scene_monitor
    assert monitor["publish_robot_description"] is False
    assert monitor["publish_robot_description_semantic"] is False


def test_rviz_uses_motion_planning_as_a_display_not_a_legacy_panel() -> None:
    configuration = yaml.safe_load(
        (SOURCE_DIR / "rviz" / "moveit.rviz").read_text(encoding="utf-8")
    )
    panel_classes = {panel["Class"] for panel in configuration["Panels"]}
    display_classes = {
        display["Class"] for display in configuration["Visualization Manager"]["Displays"]
    }
    assert "moveit_rviz_plugin/MotionPlanning" not in panel_classes
    assert "moveit_rviz_plugin/MotionPlanning" in display_classes


def test_rail_range_serves_the_full_clear_width_of_every_lane() -> None:
    """Every shelf lane, including the outermost pair, must be inside the rail's travel."""
    # The rail must reach more than a lane's centre line. An insert drives the gripper down the
    # lane between two dividers, and the planner may use any carriage position whose gripper still
    # fits that clear span, so the whole span must be inside the rail's travel. The clear span is
    # the divider pitch less one divider thickness; half of it is the lateral slack each lane
    # needs on either side of its centre.
    #
    # Measured with an independent forward-kinematic solution (rail plus the six arm joints,
    # MoveIt's margined position limits, and the surveyed workcell collision boxes): every lane's
    # pre-insert and insert poses solve, and the collision-free carriage window is +-0.085 m for a
    # can and +-0.125 m for a large bottle, centred on the lane. The window has the same width and
    # limiting link at all six lanes, and lane_01/lane_06 leave 0.555 m of rail travel unused at
    # the far edge of that window.
    rail = _load_yaml("joint_limits.yaml")["joint_limits"]["rail_joint"]
    geometry = yaml.safe_load(
        (DESCRIPTION_SOURCE / "config" / "workcell_geometry.yaml").read_text(encoding="utf-8")
    )
    dividers = sorted(float(value) for value in geometry["shelf"]["divider_x_m"].values())
    pitches = {round(right - left, 9) for left, right in zip(dividers, dividers[1:], strict=False)}
    assert len(pitches) == 1, "lanes are assumed to share one divider pitch"
    clear_span = pitches.pop() - float(geometry["shelf"]["divider_thickness_m"])
    required_slack = clear_span / 2.0

    for lane_id, lane in geometry["lanes"].items():
        center = float(lane["center_x_m"])
        assert center - required_slack >= rail["min_position"], lane_id
        assert center + required_slack <= rail["max_position"], lane_id


def test_rail_range_serves_the_full_width_of_the_stock_region() -> None:
    """Every product position a scenario can put on the tray must be inside the rail's travel."""
    # The sibling test above covers destination lanes; this covers the source side. A fixed
    # scenario stands its products on three surveyed x positions inside +-0.42 m, but a seeded
    # scenario draws them anywhere in the stock tray usable volume (owned by the description), so
    # the whole volume must be inside the rail's travel.
    #
    # The whole usable half width is required, not the drawable half width: the generator insets
    # its draws by the product radius and a wall margin, both defined in a scenario file, which a
    # scenario could loosen.
    #
    # At the surveyed geometry the tray spans +-0.68 m against a rail that travels +-1.68 m, so
    # nothing on the tray is out of reach, which is why task selection scores rail travel with no
    # reachability term. This test catches a widened tray or shortened rail changing that.
    rail = _load_yaml("joint_limits.yaml")["joint_limits"]["rail_joint"]
    geometry = yaml.safe_load(
        (DESCRIPTION_SOURCE / "config" / "workcell_geometry.yaml").read_text(encoding="utf-8")
    )
    volume = geometry["stock_tray"]["usable_volume"]
    # The tray is surveyed in the shelf frame and the rail travels in world x. Every scenario
    # stands the workcell at world x = 0 with no yaw (the only case scenario_random generates
    # placements for), so the two x axes coincide.
    center_x = float(volume["center_xyz_m"][0])
    half_width = float(volume["size_xyz_m"][0]) / 2.0
    assert center_x - half_width >= rail["min_position"]
    assert center_x + half_width <= rail["max_position"]
