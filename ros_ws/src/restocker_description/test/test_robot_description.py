# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contract tests for the expanded robot description."""

import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml

SOURCE_DIR = Path(os.environ["RESTOCKER_DESCRIPTION_SOURCE_DIR"])
XACRO_FILE = SOURCE_DIR / "urdf" / "restocker.urdf.xacro"
PLANNING_XACRO_FILE = SOURCE_DIR / "urdf" / "restocker_planning.urdf.xacro"
WORKCELL_XACRO_FILE = SOURCE_DIR / "urdf" / "workcell.urdf.xacro"
GRIPPER_GEOMETRY_FILE = SOURCE_DIR / "config" / "gripper_geometry.yaml"
PRODUCT_CATALOG_FILE = SOURCE_DIR / "config" / "product_collision_catalog.yaml"
WORKCELL_GEOMETRY_FILE = SOURCE_DIR / "config" / "workcell_geometry.yaml"

# world_from_shelf of the shipped baseline scenario: restocker_gazebo/config/
# baseline_products.yaml `workcell_pose`. Copied, not imported: a description test must not
# depend on the simulator package (same rule as PREGRASP_DISTANCE_M below).
WORKCELL_POSE = (0.0, 0.55, 0.75, 0.0, 0.0, 0.0)

EXPECTED_LINKS = {
    "world",
    "rail_base",
    "carriage",
    "base_link",
    "base_link_inertia",
    "base",
    "shoulder_link",
    "beanie",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "ft_frame",
    "flange",
    "tool0",
    "gripper",
    "grasp_center",
    "left_finger",
    "right_finger",
    "wrist_camera_link",
    "wrist_camera_optical_frame",
}

EXPECTED_JOINT_TYPES = {
    "rail_base_mount_joint": "fixed",
    "rail_joint": "prismatic",
    "base_joint": "fixed",
    "base_link-base_link_inertia": "fixed",
    "shoulder_pan_joint": "revolute",
    "shoulder_lift_joint": "revolute",
    "elbow_joint": "revolute",
    "wrist_1_joint": "revolute",
    "wrist_2_joint": "revolute",
    "wrist_3_joint": "revolute",
    "wrist_3_link-ft_frame": "fixed",
    "base_link-base_fixed_joint": "fixed",
    "wrist_3-flange": "fixed",
    "flange-tool0": "fixed",
    "gripper_mount_joint": "fixed",
    "grasp_center_joint": "fixed",
    "left_finger_joint": "prismatic",
    "right_finger_joint": "prismatic",
    "wrist_camera_mount_joint": "fixed",
    "wrist_camera_optical_joint": "fixed",
    "beanie_joint": "fixed",
}

EXPECTED_AXES = {
    "rail_joint": (1.0, 0.0, 0.0),
    "shoulder_pan_joint": (0.0, 0.0, 1.0),
    "shoulder_lift_joint": (0.0, 0.0, 1.0),
    "elbow_joint": (0.0, 0.0, 1.0),
    "wrist_1_joint": (0.0, 0.0, 1.0),
    "wrist_2_joint": (0.0, 0.0, 1.0),
    "wrist_3_joint": (0.0, 0.0, 1.0),
    "left_finger_joint": (0.0, 1.0, 0.0),
    "right_finger_joint": (0.0, -1.0, 0.0),
}

PHYSICAL_LINKS = {
    "rail_base",
    "carriage",
    "base_link_inertia",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "gripper",
    "left_finger",
    "right_finger",
    "wrist_camera_link",
}

UR10E_HARDWARE_EFFORT_LIMITS = {
    "shoulder_pan_joint": 330.0,
    "shoulder_lift_joint": 330.0,
    "elbow_joint": 150.0,
    "wrist_1_joint": 54.0,
    "wrist_2_joint": 54.0,
    "wrist_3_joint": 54.0,
}

# Pre-beanie baseline (Card 027): exactly these links carry <inertial> in the expanded model,
# at these masses. The beanie adds no mass and no inertia; a new entry or a changed value is
# a physics change this card forbids.
BASELINE_MASSES = {
    "rail_base": 80.0,
    "carriage": 42.0,
    "base_link_inertia": 4.0,
    "shoulder_link": 7.369,
    "upper_arm_link": 13.051,
    "forearm_link": 3.989,
    "wrist_1_link": 2.1,
    "wrist_2_link": 1.98,
    "wrist_3_link": 0.615,
    "gripper": 1.2,
    "left_finger": 0.18,
    "right_finger": 0.18,
    "wrist_camera_link": 0.25,
}

# Pre-beanie baseline: exactly these links carry <collision>. The beanie must not join them,
# and no collision element may be named for it.
BASELINE_COLLISION_LINKS = set(BASELINE_MASSES)

# shoulder-frame y maxima, measured from the official ur_description 3.5.1 visual meshes at
# zero pose, for the arm links that can NEVER reach the beanie at any configuration. Every arm
# joint axis after shoulder_pan is parallel to the shoulder frame's y axis, so these links' y
# bands are configuration-invariant (a rotation about y preserves y). The cap-sized hat's rim
# stays above both by several millimetres. forearm_link (y <= +0.0189) and wrist_2_link's
# vertical-axis sweep (y up to +0.139 about its origin at y -0.054) DO overlap any
# cap-covering hat: the forearm-free strip (y > 0.019) is only 57 mm wide against a 153 mm
# cap. Those grazes are visual-only in extreme folds - the links must still clear
# shoulder_link's collision mesh to get there - and are accepted by the Card 027 rework
# decision; the blocking non-interference checks are the camera and self-filter tests.
BEANIE_UNREACHABLE_LINK_MAX_Y_IN_SHOULDER_FRAME = {
    "upper_arm_link": -0.0948,
    "wrist_1_link": -0.1059,
}

# The shoulder cap's own footprint in the shoulder frame (mesh bounds). The hat must cover the
# cap yet stay inside this silhouette - a hat, not a growth beside one.
SHOULDER_CAP_FOOTPRINT = {"x": (-0.0764, 0.0764), "y": (-0.0946, 0.0764)}

# restocker_perception/src/depth_obstacle_node.cpp defaults for the shoulder_link self-filter
# capsule: segment start/end in the shoulder frame, radius, plus self_filter_margin_m. Copied,
# not imported: obstacle extraction must keep swallowing the beanie's depth returns, so a hat
# that outgrows the capsule fails here rather than becoming a phantom obstacle.
SHOULDER_SELF_FILTER_CAPSULE = {
    "start": (0.0009, 0.0108, -0.0817),
    "end": (0.0009, 0.0108, 0.0939),
    "radius": 0.130,
    "margin": 0.020,
}

# The same copy for the wrist camera housing's capsule. The housing sits off the gripper
# capsule's axis (corners reach 0.138 m against the 0.118 m that capsule covers), so without
# its own entry the overhead depth reports the camera as an obstacle colliding with the end
# effector — the phantom that blocked tray-station planning once obstacle_perception composed
# depth_obstacle_node by default (Card 023).
WRIST_CAMERA_SELF_FILTER_CAPSULE = {
    "start": (0.0, -0.045, 0.0),
    "end": (0.0, 0.045, 0.0),
    "radius": 0.033,
    "margin": 0.020,
}


def _expand_xacro(prefix: str = "", wrist_camera_sensor: bool = True) -> str:
    command = ["xacro", str(XACRO_FILE)]
    if prefix:
        command.append(f"prefix:={prefix}")
    if not wrist_camera_sensor:
        command.append("wrist_camera_sensor:=false")
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def _expand_planning_xacro() -> str:
    return subprocess.run(
        ["xacro", str(PLANNING_XACRO_FILE)], check=True, capture_output=True, text=True
    ).stdout


@pytest.fixture(scope="module")
def robot() -> ET.Element:
    return ET.fromstring(_expand_xacro())


@pytest.fixture(scope="module")
def robot_without_wrist_sensor() -> ET.Element:
    # The renderer is on by default; only the test pinning what turning it off removes needs this.
    return ET.fromstring(_expand_xacro(wrist_camera_sensor=False))


@pytest.fixture(scope="module")
def workcell() -> ET.Element:
    return ET.fromstring(
        subprocess.run(
            ["xacro", str(WORKCELL_XACRO_FILE)], check=True, capture_output=True, text=True
        ).stdout
    )


def _vector(value: str) -> tuple[float, float, float]:
    parsed = tuple(float(component) for component in value.split())
    assert len(parsed) == 3
    return parsed


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> tuple[tuple[float, ...], ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def test_expanded_urdf_passes_check_urdf() -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(_expand_xacro())
        urdf.flush()
        result = subprocess.run(
            ["check_urdf", urdf.name], check=True, capture_output=True, text=True
        )
    assert "Successfully Parsed XML" in result.stdout


def test_planning_urdf_has_empty_world_root_and_matches_physical_model() -> None:
    visualization_robot = ET.fromstring(_expand_xacro())
    planning_robot = ET.fromstring(_expand_planning_xacro())

    planning_links = {link.attrib["name"] for link in planning_robot.findall("link")}
    planning_joints = {
        joint.attrib["name"]: joint.attrib["type"] for joint in planning_robot.findall("joint")
    }
    assert planning_links == EXPECTED_LINKS
    assert planning_joints == EXPECTED_JOINT_TYPES

    child_links = {joint.find("child").attrib["link"] for joint in planning_robot.findall("joint")}
    assert planning_links - child_links == {"world"}

    visualization_joints = {
        joint.attrib["name"]: joint for joint in visualization_robot.findall("joint")
    }
    for joint in planning_robot.findall("joint"):
        reference = visualization_joints[joint.attrib["name"]]
        assert ET.tostring(joint) == ET.tostring(reference)


def test_exact_link_and_joint_contract(robot: ET.Element) -> None:
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint.attrib["type"] for joint in robot.findall("joint")}
    assert links == EXPECTED_LINKS
    assert joints == EXPECTED_JOINT_TYPES


def test_tree_is_connected_acyclic_and_rooted_at_world(robot: ET.Element) -> None:
    links = {link.attrib["name"] for link in robot.findall("link")}
    child_to_parent: dict[str, str] = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        assert parent in links
        assert child in links
        assert child not in child_to_parent
        child_to_parent[child] = parent

    roots = links - child_to_parent.keys()
    assert roots == {"world"}
    for link in links - roots:
        visited = {link}
        current = link
        while current in child_to_parent:
            current = child_to_parent[current]
            assert current not in visited
            visited.add(current)
        assert current == "world"


def test_moving_joint_axes_and_limits(robot: ET.Element) -> None:
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    for name, expected_axis in EXPECTED_AXES.items():
        joint = joints[name]
        assert _vector(joint.find("axis").attrib["xyz"]) == expected_axis
        limit = joint.find("limit")
        assert float(limit.attrib["lower"]) < float(limit.attrib["upper"])
        assert float(limit.attrib["effort"]) > 0.0
        assert float(limit.attrib["velocity"]) > 0.0

    assert joints["left_finger_joint"].find("mimic") is None
    assert joints["right_finger_joint"].find("mimic") is None


def test_visualization_retains_released_ur10e_hardware_effort_limits(
    robot: ET.Element,
) -> None:
    """The larger Gazebo velocity-motor envelope must not leak into hardware descriptions."""
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    emitted = {
        name: float(joints[name].find("limit").attrib["effort"])
        for name in UR10E_HARDWARE_EFFORT_LIMITS
    }
    assert emitted == pytest.approx(UR10E_HARDWARE_EFFORT_LIMITS)


def test_physical_links_have_geometry_and_positive_inertia(robot: ET.Element) -> None:
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for name in PHYSICAL_LINKS:
        link = links[name]
        assert link.find("visual/geometry") is not None
        assert link.find("collision/geometry") is not None
        inertial = link.find("inertial")
        assert inertial is not None
        assert float(inertial.find("mass").attrib["value"]) > 0.0
        inertia = inertial.find("inertia")
        for diagonal in ("ixx", "iyy", "izz"):
            assert float(inertia.attrib[diagonal]) > 0.0
        for product in ("ixy", "ixz", "iyz"):
            assert math.isfinite(float(inertia.attrib[product]))


def test_ur10e_links_use_upstream_visual_and_collision_meshes(robot: ET.Element) -> None:
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    ur_links = {
        "base_link_inertia": "base",
        "shoulder_link": "shoulder",
        "upper_arm_link": "upperarm",
        "forearm_link": "forearm",
        "wrist_1_link": "wrist1",
        "wrist_2_link": "wrist2",
        "wrist_3_link": "wrist3",
    }
    for link_name, mesh_stem in ur_links.items():
        visual = links[link_name].find("visual/geometry/mesh")
        collision = links[link_name].find("collision/geometry/mesh")
        assert visual is not None
        assert collision is not None
        assert visual.attrib["filename"].endswith(
            f"/ur_description/meshes/ur10e/visual/{mesh_stem}.dae"
        )
        assert collision.attrib["filename"].endswith(
            f"/ur_description/meshes/ur10e/collision/{mesh_stem}.stl"
        )


def test_camera_optical_frame_obeys_ros_axis_convention(robot: ET.Element) -> None:
    optical_joint = next(
        joint
        for joint in robot.findall("joint")
        if joint.attrib["name"] == "wrist_camera_optical_joint"
    )
    rotation = _rotation_matrix(*_vector(optical_joint.find("origin").attrib["rpy"]))
    child_axes_in_parent = tuple(tuple(row[column] for row in rotation) for column in range(3))
    expected_axes = ((0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0))
    for actual_axis, expected_axis in zip(child_axes_in_parent, expected_axes, strict=True):
        assert actual_axis == pytest.approx(expected_axis, abs=1.0e-12)


def test_gating_the_wrist_renderer_off_removes_the_sensor_and_nothing_else(
    robot: ET.Element, robot_without_wrist_sensor: ET.Element
) -> None:
    """`wrist_camera_sensor:=false` drops the renderer and keeps the link, frames and joints."""
    # The gate acts in the description, not at the ROS bridge: Gazebo renders every sensor the SDF
    # declares whether or not anything reads the images, so `cameras:=false` alone leaves the
    # rendering running.
    assert robot.find("gazebo[@reference='wrist_camera_link']/sensor") is not None
    assert robot_without_wrist_sensor.find("gazebo[@reference='wrist_camera_link']/sensor") is None
    # The body and frames are not gated: the box sets the large bottle's clearance and the optical
    # frame is resolved through tf2.
    for description in (robot, robot_without_wrist_sensor):
        link = next(
            candidate
            for candidate in description.findall("link")
            if candidate.attrib["name"] == "wrist_camera_link"
        )
        assert _vector(link.find("collision/geometry/box").attrib["size"]) == (0.05, 0.09, 0.04)
        assert any(
            candidate.attrib["name"] == "wrist_camera_optical_frame"
            for candidate in description.findall("link")
        )
    gated_joints = {
        joint.attrib["name"]: joint.attrib["type"]
        for joint in robot_without_wrist_sensor.findall("joint")
    }
    for name in ("wrist_camera_mount_joint", "wrist_camera_optical_joint"):
        assert gated_joints[name] == "fixed"


def test_wrist_camera_sensor_adds_a_frustum_and_no_geometry(robot: ET.Element) -> None:
    """The wrist RGB-D must not change the body that sets grasp clearance."""
    # The box was sized against the large bottle; growing it breaks grasping silently.
    link = next(
        candidate
        for candidate in robot.findall("link")
        if candidate.attrib["name"] == "wrist_camera_link"
    )
    for element in ("visual", "collision"):
        box = link.find(f"{element}/geometry/box")
        assert _vector(box.attrib["size"]) == (0.05, 0.09, 0.04)

    sensor = robot.find("gazebo[@reference='wrist_camera_link']/sensor")
    assert sensor is not None
    assert sensor.attrib["type"] == "rgbd_camera"
    # Gazebo would otherwise stamp images with the sensor's scoped name, which is not a TF frame,
    # and the body link's axes differ from camera_info's pinhole model.
    assert sensor.findtext("gz_frame_id") == "wrist_camera_optical_frame"
    assert sensor.findtext("topic") == "wrist_camera"
    assert float(sensor.findtext("update_rate")) > 0.0
    # Gazebo cannot read the optical joint, so the offset is restated in the sensor pose. The two
    # must agree or images render from a point tf2 does not know about.
    optical_joint = next(
        joint
        for joint in robot.findall("joint")
        if joint.attrib["name"] == "wrist_camera_optical_joint"
    )
    sensor_pose = tuple(float(value) for value in sensor.findtext("pose").split())
    assert sensor_pose[:3] == _vector(optical_joint.find("origin").attrib["xyz"])
    assert sensor_pose[3:] == (0.0, 0.0, 0.0)


# Pre-grasp standoff from restocker_task_executor's restock_action_coordinator.yaml
# `grasp.pregrasp_distance_m`. Copied, not imported: a description test must not depend on the
# executor package.
PREGRASP_DISTANCE_M = 0.18


def test_wrist_camera_frames_the_grasp_datum_at_the_pregrasp_standoff(robot: ET.Element) -> None:
    """The wrist camera must be able to see the point the gripper is about to close on."""
    # Uses the default description: the renderer gate defaults on, so if that changes this fails
    # on the missing sensor instead of passing vacuously.
    # The camera is mounted to one side to clear the large bottle, so the grasp datum sits near an
    # edge of the image and the vertical field is the binding constraint. That field follows from
    # horizontal_fov and the aspect ratio together: widening 4:3 to 16:9 at a fixed
    # horizontal_fov narrows it and pushes the datum off the top, while the other sensor
    # assertions still pass.
    rotation, translation = _fixed_pose_in("tool0", "wrist_camera_optical_frame", robot)
    _, grasp_center = _fixed_pose_in("tool0", "grasp_center", robot)

    # The pre-grasp backs off along the approach axis (tool0 +Z), so the datum sits that much
    # further along it.
    target = (grasp_center[0], grasp_center[1], grasp_center[2] + PREGRASP_DISTANCE_M)
    offset = tuple(target[i] - translation[i] for i in range(3))
    # The rotation is tool0 <- optical, so its transpose maps a tool0 point into the camera.
    point = tuple(sum(rotation[k][i] * offset[k] for k in range(3)) for i in range(3))
    assert point[2] > 0.0, "the grasp datum is behind the wrist camera at the pre-grasp"

    sensor = robot.find("gazebo[@reference='wrist_camera_link']/sensor")
    camera = sensor.find("camera")
    width = int(camera.findtext("image/width"))
    height = int(camera.findtext("image/height"))
    focal_length = (width / 2.0) / math.tan(float(camera.findtext("horizontal_fov")) / 2.0)
    column = focal_length * point[0] / point[2] + width / 2.0
    row = focal_length * point[1] / point[2] + height / 2.0

    assert float(camera.findtext("clip/near")) < point[2] < float(camera.findtext("clip/far"))
    assert 0.0 <= column <= width, f"grasp datum projects to column {column:.1f} of {width}"
    assert 0.0 <= row <= height, f"grasp datum projects to row {row:.1f} of {height}"


def _fixed_pose_in(
    ancestor: str, frame: str, robot: ET.Element
) -> tuple[tuple[tuple[float, ...], ...], tuple[float, float, float]]:
    """Return `frame`'s rotation and origin expressed in `ancestor`, over fixed joints only."""
    children = {joint.find("child").attrib["link"]: joint for joint in robot.findall("joint")}
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    translation = (0.0, 0.0, 0.0)
    current = frame
    while current != ancestor:
        joint = children[current]
        # A movable joint would make the answer depend on a configuration this test does not set.
        assert joint.attrib["type"] == "fixed", f"{joint.attrib['name']} is not fixed"
        origin = joint.find("origin")
        step = _rotation_matrix(*_vector(origin.attrib["rpy"]))
        offset = _vector(origin.attrib["xyz"])
        translation = tuple(
            offset[i] + sum(step[i][k] * translation[k] for k in range(3)) for i in range(3)
        )
        rotation = tuple(
            tuple(sum(step[i][k] * rotation[k][j] for k in range(3)) for j in range(3))
            for i in range(3)
        )
        current = joint.find("parent").attrib["link"]
    return rotation, translation


def test_grasp_center_is_the_fixed_finger_contact_datum(robot: ET.Element) -> None:
    grasp_joint = next(
        joint for joint in robot.findall("joint") if joint.attrib["name"] == "grasp_center_joint"
    )
    assert grasp_joint.find("parent").attrib["link"] == "gripper"
    assert grasp_joint.find("child").attrib["link"] == "grasp_center"
    assert _vector(grasp_joint.find("origin").attrib["xyz"]) == (0.0, 0.0, 0.14)
    assert _vector(grasp_joint.find("origin").attrib["rpy"]) == (0.0, 0.0, 0.0)


def test_gripper_geometry_config_is_the_emitted_urdf_source(robot: ET.Element) -> None:
    geometry = yaml.safe_load(GRIPPER_GEOMETRY_FILE.read_text(encoding="utf-8"))
    assert geometry["schema_version"] == 1
    assert set(geometry) == {
        "schema_version",
        "body",
        "grasp_center",
        "finger",
        "attachment",
    }

    links = {link.attrib["name"]: link for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    body = geometry["body"]
    assert _vector(links["gripper"].find("collision/origin").attrib["xyz"]) == pytest.approx(
        body["center_xyz_m"]
    )
    assert _vector(links["gripper"].find("collision/geometry/box").attrib["size"]) == (
        pytest.approx(body["size_xyz_m"])
    )
    assert float(links["gripper"].find("inertial/mass").attrib["value"]) == pytest.approx(
        body["mass_kg"]
    )

    grasp = geometry["grasp_center"]
    assert _vector(joints["grasp_center_joint"].find("origin").attrib["xyz"]) == pytest.approx(
        grasp["xyz_m"]
    )
    assert _vector(joints["grasp_center_joint"].find("origin").attrib["rpy"]) == pytest.approx(
        grasp["rpy_rad"]
    )

    finger = geometry["finger"]
    for side in ("left", "right"):
        joint = joints[f"{side}_finger_joint"]
        link = links[f"{side}_finger"]
        assert _vector(joint.find("origin").attrib["xyz"]) == pytest.approx(
            finger[side]["origin_xyz_m"]
        )
        assert _vector(joint.find("axis").attrib["xyz"]) == pytest.approx(finger[side]["axis"])
        assert _vector(link.find("collision/origin").attrib["xyz"]) == pytest.approx(
            finger["center_xyz_m"]
        )
        assert _vector(link.find("collision/geometry/box").attrib["size"]) == pytest.approx(
            finger["size_xyz_m"]
        )
        assert float(link.find("inertial/mass").attrib["value"]) == pytest.approx(
            finger["mass_kg"]
        )

        limits = finger["joint"]
        emitted_limit = joint.find("limit")
        assert float(emitted_limit.attrib["lower"]) == pytest.approx(limits["lower_m"])
        assert float(emitted_limit.attrib["upper"]) == pytest.approx(limits["upper_m"])
        assert float(emitted_limit.attrib["effort"]) == pytest.approx(limits["effort_n"])
        assert float(emitted_limit.attrib["velocity"]) == pytest.approx(limits["velocity_mps"])
        emitted_dynamics = joint.find("dynamics")
        assert float(emitted_dynamics.attrib["damping"]) == pytest.approx(
            limits["damping_n_s_per_m"]
        )
        assert float(emitted_dynamics.attrib["friction"]) == pytest.approx(limits["friction_n"])


def test_gripper_clearance_policy_fits_every_catalog_product() -> None:
    geometry = yaml.safe_load(GRIPPER_GEOMETRY_FILE.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(PRODUCT_CATALOG_FILE.read_text(encoding="utf-8"))
    finger = geometry["finger"]
    limits = finger["joint"]
    policy = geometry["attachment"]

    finger_half_width = 0.5 * float(finger["size_xyz_m"][1])
    left_inner_at_zero = float(finger["left"]["origin_xyz_m"][1]) - finger_half_width
    right_inner_at_zero = float(finger["right"]["origin_xyz_m"][1]) + finger_half_width
    zero_gap = left_inner_at_zero - right_inner_at_zero
    maximum_gap = zero_gap + 2.0 * float(limits["upper_m"])
    clearance = float(policy["minimum_inner_clearance_m"])
    hold_clearance = float(policy["hold_clearance_per_side_m"])
    open_clearance = float(policy["open_clearance_per_side_m"])

    assert 0.0 < zero_gap < maximum_gap
    assert clearance > 0.0
    assert clearance <= hold_clearance < open_clearance
    assert float(policy["minimum_contact_overlap_m"]) > 0.0
    # Strictly inside the stroke: parking the jaws on the joint limit through a full arm traverse
    # pushed them past it, the world state rejected the telemetry and the next gripper trajectory
    # aborted on its path tolerance.
    assert float(policy["open_target_m"]) < float(limits["upper_m"])
    for product in catalog["geometries"]:
        diameter = 2.0 * float(product["shape"]["radius_m"])
        required_gap = diameter + 2.0 * hold_clearance
        hold_target = 0.5 * (required_gap - zero_gap)
        assert required_gap <= maximum_gap
        assert float(limits["lower_m"]) <= hold_target <= float(limits["upper_m"])


def test_prefix_applies_to_every_robot_link_and_joint() -> None:
    prefixed = ET.fromstring(_expand_xacro("test_"))
    links = {link.attrib["name"] for link in prefixed.findall("link")}
    joints = {joint.attrib["name"] for joint in prefixed.findall("joint")}
    assert links == {"world"} | {f"test_{name}" for name in EXPECTED_LINKS - {"world"}}
    assert joints == {f"test_{name}" for name in EXPECTED_JOINT_TYPES}


def _matmul(a: list, b: list) -> list:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _matvec(a: list, v: tuple) -> tuple:
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _rotation_about_axis(axis: tuple, angle: float) -> list:
    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / length, y / length, z / length
    c, s, C = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ]


def _link_pose(
    robot: ET.Element, target: str, positions: dict[str, float]
) -> tuple[list, tuple[float, float, float]]:
    """Pose of `target` in the URDF root frame; movable joints read `positions` (else zero)."""
    joint_by_child = {
        joint.find("child").attrib["link"]: joint for joint in robot.findall("joint")
    }
    chain = []
    current = target
    while current in joint_by_child:
        joint = joint_by_child[current]
        chain.append(joint)
        current = joint.find("parent").attrib["link"]
    rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    translation = (0.0, 0.0, 0.0)
    for joint in reversed(chain):
        origin = joint.find("origin")
        origin_rotation = _rotation_matrix(*_vector(origin.attrib.get("rpy", "0 0 0")))
        origin_translation = _vector(origin.attrib.get("xyz", "0 0 0"))
        name = joint.attrib["name"]
        step_rotation = origin_rotation
        step_translation = origin_translation
        if joint.attrib["type"] in ("revolute", "continuous", "prismatic") and name in positions:
            angle = positions[name]
            axis = _vector(joint.find("axis").attrib["xyz"])
            if joint.attrib["type"] == "prismatic":
                motion = tuple(axis[i] * angle for i in range(3))
                step_translation = tuple(
                    origin_translation[i]
                    + sum(origin_rotation[i][k] * motion[k] for k in range(3))
                    for i in range(3)
                )
            else:
                step_rotation = _matmul(origin_rotation, _rotation_about_axis(axis, angle))
        translation = tuple(
            translation[i] + sum(rotation[i][k] * step_translation[k] for k in range(3))
            for i in range(3)
        )
        rotation = _matmul(rotation, step_rotation)
    return [list(row) for row in rotation], translation


def _invert_pose(rotation: list, translation: tuple) -> tuple[list, tuple]:
    inverse = [[rotation[j][i] for j in range(3)] for i in range(3)]
    origin = tuple(-sum(inverse[i][k] * translation[k] for k in range(3)) for i in range(3))
    return inverse, origin


def _transform_point(
    rotation: list, translation: tuple, point: tuple
) -> tuple[float, float, float]:
    return tuple(
        translation[i] + sum(rotation[i][k] * point[k] for k in range(3)) for i in range(3)
    )


def _beanie_bounds_in_link(robot: ET.Element) -> tuple[tuple, tuple]:
    """Axis-aligned bounds of every beanie visual, in the beanie link frame."""
    link = robot.find("link[@name='beanie']")
    assert link is not None
    low = [math.inf] * 3
    high = [-math.inf] * 3
    for visual in link.findall("visual"):
        origin = visual.find("origin")
        rpy = _vector(origin.attrib.get("rpy", "0 0 0")) if origin is not None else (0.0,) * 3
        assert rpy == (0.0, 0.0, 0.0), "beanie visuals are authored axis-aligned"
        xyz = _vector(origin.attrib.get("xyz", "0 0 0")) if origin is not None else (0.0,) * 3
        geometry = visual.find("geometry")
        if geometry.find("cylinder") is not None:
            cylinder = geometry.find("cylinder")
            half = (
                float(cylinder.attrib["radius"]),
                float(cylinder.attrib["radius"]),
                float(cylinder.attrib["length"]) / 2.0,
            )
        elif geometry.find("sphere") is not None:
            radius = float(geometry.find("sphere").attrib["radius"])
            half = (radius, radius, radius)
        elif geometry.find("box") is not None:
            size = _vector(geometry.find("box").attrib["size"])
            half = tuple(component / 2.0 for component in size)
        else:  # pragma: no cover - the beanie is primitives by design
            raise AssertionError(f"unexpected beanie geometry: {ET.tostring(geometry)}")
        for axis in range(3):
            low[axis] = min(low[axis], xyz[axis] - half[axis])
            high[axis] = max(high[axis], xyz[axis] + half[axis])
    assert all(math.isfinite(value) for value in low + high)
    return tuple(low), tuple(high)


def _beanie_visual_spheres(robot: ET.Element) -> list[tuple[tuple, float]]:
    """One exact bounding sphere per beanie visual, in the beanie link frame."""
    # Per-visual spheres stay tight around a wide, flat hat where a single sphere around the
    # union box would be nearly twice the radius and drown the frustum margins.
    link = robot.find("link[@name='beanie']")
    assert link is not None
    spheres = []
    for visual in link.findall("visual"):
        origin = visual.find("origin")
        rpy = _vector(origin.attrib.get("rpy", "0 0 0")) if origin is not None else (0.0,) * 3
        assert rpy == (0.0, 0.0, 0.0), "beanie visuals are authored axis-aligned"
        xyz = _vector(origin.attrib.get("xyz", "0 0 0")) if origin is not None else (0.0,) * 3
        geometry = visual.find("geometry")
        if geometry.find("sphere") is not None:
            radius = float(geometry.find("sphere").attrib["radius"])
        elif geometry.find("cylinder") is not None:
            cylinder = geometry.find("cylinder")
            radius = math.hypot(
                float(cylinder.attrib["radius"]), float(cylinder.attrib["length"]) / 2.0
            )
        elif geometry.find("box") is not None:
            size = _vector(geometry.find("box").attrib["size"])
            radius = math.sqrt(sum(component * component for component in size)) / 2.0
        else:  # pragma: no cover - the beanie is primitives by design
            raise AssertionError(f"unexpected beanie geometry: {ET.tostring(geometry)}")
        spheres.append((xyz, radius))
    assert spheres
    return spheres


def _beanie_surface_points(robot: ET.Element) -> list[tuple]:
    """Dense sample of every beanie visual's surface, in the beanie link frame."""
    link = robot.find("link[@name='beanie']")
    assert link is not None
    points = []
    for visual in link.findall("visual"):
        origin = visual.find("origin")
        xyz = _vector(origin.attrib.get("xyz", "0 0 0")) if origin is not None else (0.0,) * 3
        geometry = visual.find("geometry")
        if geometry.find("sphere") is not None:
            radius = float(geometry.find("sphere").attrib["radius"])
            for latitude in range(33):
                polar = math.pi * latitude / 32.0
                for longitude in range(64):
                    azimuth = 2.0 * math.pi * longitude / 64.0
                    points.append(
                        (
                            xyz[0] + radius * math.sin(polar) * math.cos(azimuth),
                            xyz[1] + radius * math.sin(polar) * math.sin(azimuth),
                            xyz[2] + radius * math.cos(polar),
                        )
                    )
        elif geometry.find("cylinder") is not None:
            cylinder = geometry.find("cylinder")
            radius = float(cylinder.attrib["radius"])
            half_length = float(cylinder.attrib["length"]) / 2.0
            for height in (-half_length, -half_length / 2.0, 0.0, half_length / 2.0, half_length):
                for step in range(64):
                    azimuth = 2.0 * math.pi * step / 64.0
                    points.append(
                        (
                            xyz[0] + radius * math.cos(azimuth),
                            xyz[1] + radius * math.sin(azimuth),
                            xyz[2] + height,
                        )
                    )
        else:  # pragma: no cover - the beanie is primitives by design
            raise AssertionError(f"unexpected beanie geometry: {ET.tostring(geometry)}")
    assert points
    return points


def _frustum_plane_distances(
    centre_in_optical: tuple,
    horizontal_fov: float,
    near: float,
    far: float,
    width: int,
    height: int,
) -> list[float]:
    """Signed distances from the sphere centre to each inward frustum plane."""
    # A plane with distance less than minus the sphere radius separates the sphere from the
    # frustum entirely; the callers compare against the radius they already hold.
    focal = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    half_h = horizontal_fov / 2.0
    # Vertical half-FOV follows from the focal length and the aspect ratio.
    half_v = math.atan((height / 2.0) / focal)
    x, y, z = centre_in_optical
    planes = (
        (0.0, 0.0, 1.0, near),
        (0.0, 0.0, -1.0, -far),
        (-math.cos(half_h), 0.0, math.sin(half_h), 0.0),
        (math.cos(half_h), 0.0, math.sin(half_h), 0.0),
        (0.0, -math.cos(half_v), math.sin(half_v), 0.0),
        (0.0, math.cos(half_v), math.sin(half_v), 0.0),
    )
    return [nx * x + ny * y + nz * z - offset for nx, ny, nz, offset in planes]


def _distance_point_to_segment(point: tuple, start: tuple, end: tuple) -> float:
    ab = tuple(end[axis] - start[axis] for axis in range(3))
    ap = tuple(point[axis] - start[axis] for axis in range(3))
    denominator = sum(component * component for component in ab)
    parameter = (
        0.0
        if denominator == 0.0
        else max(0.0, min(1.0, sum(ap[axis] * ab[axis] for axis in range(3)) / denominator))
    )
    closest = tuple(start[axis] + parameter * ab[axis] for axis in range(3))
    return math.sqrt(sum((point[axis] - closest[axis]) ** 2 for axis in range(3)))


def test_beanie_is_visual_only_and_fixed_to_the_shoulder(robot: ET.Element) -> None:
    """SC-001: a dedicated link with visuals, no collision, no mass, on a fixed joint."""
    link = robot.find("link[@name='beanie']")
    assert link is not None
    visuals = link.findall("visual")
    assert len(visuals) == 5
    for visual in visuals:
        assert visual.find("geometry") is not None
        assert visual.find("material") is not None
    assert link.find("collision") is None
    assert link.find("inertial") is None

    joint = robot.find("joint[@name='beanie_joint']")
    assert joint is not None
    assert joint.attrib["type"] == "fixed"
    assert joint.find("parent").attrib["link"] == "shoulder_link"
    assert joint.find("child").attrib["link"] == "beanie"
    origin = joint.find("origin")
    assert _vector(origin.attrib["xyz"]) == pytest.approx((0.0, -0.010, 0.090))
    assert _vector(origin.attrib["rpy"]) == pytest.approx((-0.087, 0.0, 0.0))


def test_no_collision_is_named_for_the_beanie_and_the_parent_collision_set_is_unchanged(
    robot: ET.Element,
) -> None:
    """SC-002: no collision is named for the beanie and the parent's collision set is intact."""
    collision_links = {
        link.attrib["name"] for link in robot.findall("link") if link.find("collision") is not None
    }
    assert collision_links == BASELINE_COLLISION_LINKS
    for link in robot.findall("link"):
        assert "beanie" not in link.attrib["name"] or link.find("collision") is None
        for collision in link.findall("collision"):
            assert "beanie" not in collision.attrib.get("name", "").lower()

    shoulder = robot.find("link[@name='shoulder_link']")
    collisions = shoulder.findall("collision")
    assert len(collisions) == 1
    mesh = collisions[0].find("geometry/mesh")
    assert mesh is not None
    assert mesh.attrib["filename"].endswith("/ur_description/meshes/ur10e/collision/shoulder.stl")


def test_masses_and_inertias_match_the_pre_beanie_baseline(robot: ET.Element) -> None:
    """SC-002: masses and inertias unchanged - same links carry inertia, at the same values."""
    masses = {
        link.attrib["name"]: float(link.find("inertial/mass").attrib["value"])
        for link in robot.findall("link")
        if link.find("inertial") is not None
    }
    assert set(masses) == set(BASELINE_MASSES)
    for name, expected in BASELINE_MASSES.items():
        assert masses[name] == pytest.approx(expected), name
    assert robot.find("link[@name='beanie']/inertial") is None


def test_beanie_covers_the_cap_and_stays_out_of_the_upper_arm_and_wrist_bands(
    robot: ET.Element,
) -> None:
    """Cover the cap inside its silhouette, clear of the links that can never reach it."""
    # y/x of a beanie-frame point expressed in the shoulder frame, via the fixed joint pose.
    # Exact per-primitive extents along a row of that rigid pose (row is a unit vector).
    beanie_rotation, beanie_translation = _link_pose(robot, "beanie", {})
    shoulder_rotation, shoulder_translation = _link_pose(robot, "shoulder_link", {})
    shoulder_inverse, shoulder_origin = _invert_pose(shoulder_rotation, shoulder_translation)
    relative_rotation = _matmul(shoulder_inverse, beanie_rotation)
    relative_translation = _transform_point(shoulder_inverse, shoulder_origin, beanie_translation)
    link = robot.find("link[@name='beanie']")

    def extent_along(row_index: int) -> tuple[float, float]:
        row = [relative_rotation[row_index][axis] for axis in range(3)]
        offset = relative_translation[row_index]
        low = math.inf
        high = -math.inf
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            xyz = _vector(origin.attrib.get("xyz", "0 0 0")) if origin is not None else (0.0,) * 3
            geometry = visual.find("geometry")
            base = offset + sum(row[axis] * xyz[axis] for axis in range(3))
            if geometry.find("sphere") is not None:
                span = float(geometry.find("sphere").attrib["radius"])
            elif geometry.find("cylinder") is not None:
                cylinder = geometry.find("cylinder")
                span = float(cylinder.attrib["radius"]) * math.hypot(row[0], row[1]) + (
                    float(cylinder.attrib["length"]) / 2.0
                ) * abs(row[2])
            elif geometry.find("box") is not None:
                size = _vector(geometry.find("box").attrib["size"])
                span = sum(abs(row[axis]) * size[axis] / 2.0 for axis in range(3))
            else:  # pragma: no cover
                raise AssertionError("unexpected beanie geometry")
            low = min(low, base - span)
            high = max(high, base + span)
        return low, high

    y_low, y_high = extent_along(1)
    x_low, x_high = extent_along(0)

    # Links whose configuration-invariant bands never reach the hat: clearance must hold with
    # a real margin, so a geometry change that creeps toward the upper arm fails here.
    unreachable_max = max(BEANIE_UNREACHABLE_LINK_MAX_Y_IN_SHOULDER_FRAME.values())
    margin = 0.005
    assert y_low >= unreachable_max + margin, (
        f"beanie y in shoulder frame reaches {y_low:.4f}; upper_arm/wrist_1 bands top out at "
        f"{unreachable_max:.4f}"
    )
    # Covers the cap yet stays inside the shoulder's own silhouette (no sideways overhang).
    footprint = SHOULDER_CAP_FOOTPRINT
    assert x_low >= footprint["x"][0], f"beanie overhangs the cap in -x: {x_low:.4f}"
    assert x_high <= footprint["x"][1], f"beanie overhangs the cap in +x: {x_high:.4f}"
    assert y_low >= footprint["y"][0], f"beanie overhangs the cap in -y: {y_low:.4f}"
    assert y_high <= footprint["y"][1], f"beanie overhangs the cap in +y: {y_high:.4f}"
    # Wide enough to read as a hat: the cuff must span at least half the cap's width.
    assert x_high - x_low >= 0.5 * (footprint["x"][1] - footprint["x"][0])


def test_beanie_stays_inside_the_shoulder_self_filter_capsule(robot: ET.Element) -> None:
    """Depth returns from the beanie must keep being discarded as the robot's own surface."""
    # Sampled surface points rather than one union sphere: a cap-wide hat's union sphere is
    # ~0.11 m and would drown the 0.150 m capsule, while no actual hat point comes close.
    beanie_rotation, beanie_translation = _link_pose(robot, "beanie", {})
    shoulder_rotation, shoulder_translation = _link_pose(robot, "shoulder_link", {})
    inverse, origin = _invert_pose(shoulder_rotation, shoulder_translation)

    capsule = SHOULDER_SELF_FILTER_CAPSULE
    effective_radius = capsule["radius"] + capsule["margin"]
    worst = 0.0
    for point in _beanie_surface_points(robot):
        point_shoulder = _transform_point(
            inverse, origin, _transform_point(beanie_rotation, beanie_translation, point)
        )
        worst = max(
            worst,
            _distance_point_to_segment(point_shoulder, capsule["start"], capsule["end"]),
        )
    assert worst <= effective_radius, (
        f"beanie surface reaches {worst:.4f} m from the capsule axis, outside the "
        f"{effective_radius:.4f} m self-filter capsule"
    )
    # The claim is only useful with clearance left: require at least 20 mm of margin so a
    # growth of the hat or a shrink of the filter fails rather than grazing the bound.
    assert worst <= effective_radius - 0.020


def test_wrist_camera_body_stays_inside_its_self_filter_capsule(robot: ET.Element) -> None:
    """Depth returns from the camera housing must keep being discarded as the robot's own."""
    link = next(
        candidate
        for candidate in robot.findall("link")
        if candidate.attrib["name"] == "wrist_camera_link"
    )
    collision = link.find("collision")
    assert collision is not None, "the wrist camera body must carry a collision box"
    box = collision.find("geometry/box")
    assert box is not None, "the wrist camera body must be a box"
    size = _vector(box.attrib["size"])
    origin = collision.find("origin")
    if origin is not None and "xyz" in origin.attrib:
        centre = _vector(origin.attrib["xyz"])
    else:
        centre = (0.0, 0.0, 0.0)

    capsule = WRIST_CAMERA_SELF_FILTER_CAPSULE
    effective_radius = capsule["radius"] + capsule["margin"]
    worst = 0.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corner = (
                    centre[0] + sx * size[0] / 2.0,
                    centre[1] + sy * size[1] / 2.0,
                    centre[2] + sz * size[2] / 2.0,
                )
                worst = max(
                    worst,
                    _distance_point_to_segment(corner, capsule["start"], capsule["end"]),
                )
    assert worst <= effective_radius, (
        f"wrist camera body reaches {worst:.4f} m from the capsule axis, outside the "
        f"{effective_radius:.4f} m self-filter capsule"
    )
    # Same 20 mm of clearance as the beanie bound: the housing or the capsule growing into a
    # graze fails here rather than becoming a phantom obstacle under obstacle_perception.
    assert worst <= effective_radius - 0.020


def _clear_of_frustum(
    sphere_world: tuple[tuple, float],
    optical_pose: tuple[list, tuple],
    horizontal_fov: float,
    near: float,
    far: float,
    width: int,
    height: int,
) -> list[float]:
    """Plane distances of the sphere centre for the given world -> optical pose."""
    centre, _radius = sphere_world
    rotation, translation = optical_pose
    inverse, origin = _invert_pose(rotation, translation)
    centre_optical = _transform_point(inverse, origin, centre)
    return _frustum_plane_distances(centre_optical, horizontal_fov, near, far, width, height)


def test_wrist_camera_frustum_is_clear_of_the_beanie(robot: ET.Element) -> None:
    """SC-003: the wrist RGB-D frustum is clear of the beanie at the viewing and start poses."""
    sensor = robot.find("gazebo[@reference='wrist_camera_link']/sensor")
    camera = sensor.find("camera")
    horizontal_fov = float(camera.findtext("horizontal_fov"))
    width = int(camera.findtext("image/width"))
    height = int(camera.findtext("image/height"))
    near = float(camera.findtext("clip/near"))
    far = float(camera.findtext("clip/far"))

    poses = (
        # Default description pose: what RViz and joint_state_publisher start at.
        ("default pose", {}),
        # Gazebo's declared initial arm state (restocker_sim / controllers initial_value).
        ("simulator initial pose", {"shoulder_lift_joint": -1.57, "wrist_1_joint": -1.57}),
    )
    for label, positions in poses:
        optical = _link_pose(robot, "wrist_camera_optical_frame", positions)
        rotation_arm, translation_arm = _link_pose(robot, "beanie", positions)
        for centre_link, radius in _beanie_visual_spheres(robot):
            centre_world = _transform_point(rotation_arm, translation_arm, centre_link)
            distances = _clear_of_frustum(
                (centre_world, radius), optical, horizontal_fov, near, far, width, height
            )
            assert any(distance < -radius for distance in distances), (
                f"a beanie visual (r={radius:.3f}) intersects the wrist frustum at the "
                f"{label}: {distances}"
            )


def test_overhead_camera_regions_stay_clear_of_the_beanie_image(
    robot: ET.Element, workcell: ET.Element
) -> None:
    """SC-003: the beanie's overhead image never overlaps a surveyed work region's image."""
    # The overhead camera sees the robot - the frustum is aimed over it - so the claim that
    # matters for perception is that the hat stays in the empty image band between the lane
    # band and the tray band, where no lane depth or product detection is ever scored.
    # Projecting each region's corners bounds its image (the regions are convex and every
    # corner is in front of the camera); the same is done for the beanie's bounding box, then
    # the two boxes must never overlap, over the full rail and shoulder-pan sweep.
    geometry = yaml.safe_load(WORKCELL_GEOMETRY_FILE.read_text(encoding="utf-8"))
    mount = workcell.find("joint[@name='overhead_camera_mount_joint']")
    optical_joint = workcell.find("joint[@name='overhead_camera_optical_joint']")

    def _fixed_transform(joint: ET.Element) -> tuple[list, tuple]:
        origin = joint.find("origin")
        return (
            _rotation_matrix(*_vector(origin.attrib.get("rpy", "0 0 0"))),
            _vector(origin.attrib.get("xyz", "0 0 0")),
        )

    mount_rotation, mount_translation = _fixed_transform(mount)
    optical_rotation, optical_translation = _fixed_transform(optical_joint)
    shelf_rotation = _matmul(mount_rotation, optical_rotation)
    shelf_translation = _transform_point(mount_rotation, mount_translation, optical_translation)
    # world <- shelf (WORKCELL_POSE has zero rotation in the shipped baseline), then compose.
    world_from_shelf_t = WORKCELL_POSE[:3]
    optical_translation_world = tuple(
        world_from_shelf_t[i] + shelf_translation[i] for i in range(3)
    )

    sensor = workcell.find("gazebo[@reference='overhead_camera_link']/sensor")
    camera = sensor.find("camera")
    horizontal_fov = float(camera.findtext("horizontal_fov"))
    width = int(camera.findtext("image/width"))
    height = int(camera.findtext("image/height"))
    near = float(camera.findtext("clip/near"))
    far = float(camera.findtext("clip/far"))
    focal = (width / 2.0) / math.tan(horizontal_fov / 2.0)

    def project(point_world: tuple) -> tuple[float, float, float]:
        offset = tuple(point_world[i] - optical_translation_world[i] for i in range(3))
        # shelf_rotation is shelf <- optical; the baseline world pose has no rotation, so it
        # is also world <- optical. Its transpose maps a world point into the camera.
        camera_point = _matvec(
            [[shelf_rotation[j][i] for j in range(3)] for i in range(3)], offset
        )
        depth = camera_point[2]
        assert depth > 0.0, f"{point_world} is behind the overhead camera"
        return (
            focal * camera_point[0] / depth + width / 2.0,
            focal * camera_point[1] / depth + height / 2.0,
            depth,
        )

    regions: dict[str, list[tuple]] = {}
    volume = geometry["stock_tray"]["usable_volume"]
    centre, size = volume["center_xyz_m"], volume["size_xyz_m"]
    regions["stock_tray"] = [
        tuple(
            WORKCELL_POSE[axis] + centre[axis] + sign * size[axis] / 2.0
            for axis, sign in ((0, sx), (1, sy), (2, sz))
        )
        for sx in (-1, 1)
        for sy in (-1, 1)
        for sz in (-1, 1)
    ]
    for lane_id, lane in geometry["lanes"].items():
        half_width = lane["usable_width_m"] / 2.0
        regions[lane_id] = [
            (
                WORKCELL_POSE[0] + lane["center_x_m"] + sign * half_width,
                WORKCELL_POSE[1] + y,
                WORKCELL_POSE[2] + z,
            )
            for sign in (-1, 1)
            for y in (
                lane["rear_clearance_m"],
                lane["rear_clearance_m"] + lane["usable_depth_m"],
            )
            for z in (
                lane["floor_clearance_m"],
                lane["floor_clearance_m"] + lane["usable_height_m"],
            )
        ]

    region_boxes = {}
    for name, corners in regions.items():
        projected = [project(corner) for corner in corners]
        assert all(near < depth < far for _, _, depth in projected), name
        region_boxes[name] = (
            min(point[0] for point in projected),
            max(point[0] for point in projected),
            min(point[1] for point in projected),
            max(point[1] for point in projected),
        )

    rail = next(joint for joint in robot.findall("joint") if joint.attrib["name"] == "rail_joint")
    rail_limit = rail.find("limit")
    rail_lower = float(rail_limit.attrib["lower"])
    rail_upper = float(rail_limit.attrib["upper"])
    low, high = _beanie_bounds_in_link(robot)
    box_corners = [
        (x, y, z) for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])
    ]

    checked = 0
    for rail_index in range(9):
        rail_position = rail_lower + (rail_upper - rail_lower) * rail_index / 8.0
        for pan_index in range(24):
            pan = -math.pi + 2.0 * math.pi * pan_index / 24.0
            positions = {"rail_joint": rail_position, "shoulder_pan_joint": pan}
            rotation, translation = _link_pose(robot, "beanie", positions)
            projected = [
                project(_transform_point(rotation, translation, corner)) for corner in box_corners
            ]
            assert all(near < depth < far for _, _, depth in projected)
            beanie_box = (
                min(point[0] for point in projected),
                max(point[0] for point in projected),
                min(point[1] for point in projected),
                max(point[1] for point in projected),
            )
            for name, region_box in region_boxes.items():
                overlap_u = min(beanie_box[1], region_box[1]) - max(beanie_box[0], region_box[0])
                overlap_v = min(beanie_box[3], region_box[3]) - max(beanie_box[2], region_box[2])
                assert overlap_u < 0.0 or overlap_v < 0.0, (
                    f"beanie image {beanie_box} overlaps {name} {region_box} "
                    f"at rail={rail_position:.3f} pan={pan:.3f}"
                )
            checked += 1
    assert checked == 9 * 24
