# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static tests for the Gazebo model, control interfaces, and controller partition."""

import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml

GAZEBO_SOURCE = Path(os.environ["RESTOCKER_GAZEBO_SOURCE_DIR"])
DESCRIPTION_SOURCE = Path(os.environ["RESTOCKER_DESCRIPTION_SOURCE_DIR"])
CONTROL_SOURCE = Path(os.environ["RESTOCKER_CONTROL_SOURCE_DIR"])
SIMULATION_XACRO = GAZEBO_SOURCE / "urdf" / "restocker_sim.urdf.xacro"
CONTROLLER_CONFIG = CONTROL_SOURCE / "config" / "controllers.yaml"
WORLD_FILE = GAZEBO_SOURCE / "worlds" / "restocking.sdf"
PRODUCT_XACRO = GAZEBO_SOURCE / "urdf" / "product.urdf.xacro"
SCENARIO_DIR = GAZEBO_SOURCE / "config"
PRODUCT_SCENARIO = SCENARIO_DIR / "baseline_products.yaml"
SAME_SHAPE_SCENARIO = SCENARIO_DIR / "same_shape_products.yaml"
TEXTURE_DIR = GAZEBO_SOURCE / "materials" / "textures"
PRODUCT_CATALOG = DESCRIPTION_SOURCE / "config" / "product_collision_catalog.yaml"

COMMAND_JOINTS = {
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

UR10E_SIMULATOR_EFFORT_ENVELOPE = {
    "shoulder_pan_joint": 2640.0,
    "shoulder_lift_joint": 2640.0,
    "elbow_joint": 1200.0,
    "wrist_1_joint": 432.0,
    "wrist_2_joint": 432.0,
    "wrist_3_joint": 432.0,
}


@pytest.fixture(scope="module")
def simulation_robot() -> ET.Element:
    expanded = subprocess.run(
        ["xacro", str(SIMULATION_XACRO), f"controller_config:={CONTROLLER_CONFIG}"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return ET.fromstring(expanded)


def test_simulation_urdf_is_valid_and_rooted_at_physical_rail(
    simulation_robot: ET.Element,
) -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(ET.tostring(simulation_robot, encoding="unicode"))
        urdf.flush()
        subprocess.run(["check_urdf", urdf.name], check=True, capture_output=True, text=True)

    links = {link.attrib["name"] for link in simulation_robot.findall("link")}
    child_links = {
        joint.find("child").attrib["link"] for joint in simulation_robot.findall("joint")
    }
    assert links - child_links == {"rail_base"}
    assert "world" not in links
    assert simulation_robot.find("joint[@name='rail_base_mount_joint']") is None


def test_control_interfaces_match_urdf_limits(simulation_robot: ET.Element) -> None:
    control = simulation_robot.find("ros2_control")
    assert control is not None
    assert control.attrib == {"name": "GazeboSimSystem", "type": "system"}
    assert control.findtext("hardware/plugin") == "gz_ros2_control/GazeboSimSystem"

    hardware_joints = {joint.attrib["name"]: joint for joint in control.findall("joint")}
    assert set(hardware_joints) == COMMAND_JOINTS
    urdf_joints = {joint.attrib["name"]: joint for joint in simulation_robot.findall("joint")}

    for name in COMMAND_JOINTS:
        hardware_joint = hardware_joints[name]
        # Both interfaces are exported for every joint; controllers.yaml decides which one each
        # controller claims, and that partition is asserted separately below.
        assert [item.attrib["name"] for item in hardware_joint.findall("command_interface")] == [
            "position",
            "velocity",
        ]
        assert {item.attrib["name"] for item in hardware_joint.findall("state_interface")} == {
            "position",
            "velocity",
        }
        command = hardware_joint.find("command_interface")
        parameters = {item.attrib["name"]: float(item.text) for item in command.findall("param")}
        urdf_limit = urdf_joints[name].find("limit")
        assert parameters == {
            "min": pytest.approx(float(urdf_limit.attrib["lower"])),
            "max": pytest.approx(float(urdf_limit.attrib["upper"])),
        }

        position_state = hardware_joint.find("state_interface[@name='position']")
        initial_values = position_state.findall("param[@name='initial_value']")
        assert len(initial_values) == 1
        initial_value = float(initial_values[0].text)
        expected_initial = {
            "shoulder_lift_joint": -1.57,
            "wrist_1_joint": -1.57,
            "left_finger_joint": 0.005,
            "right_finger_joint": 0.005,
        }.get(name, 0.0)
        assert initial_value == pytest.approx(expected_initial)
        lower = float(urdf_limit.attrib["lower"])
        upper = float(urdf_limit.attrib["upper"])
        assert lower < initial_value < upper

    left_limit = urdf_joints["left_finger_joint"].find("limit").attrib
    right_limit = urdf_joints["right_finger_joint"].find("limit").attrib
    assert left_limit == right_limit


def test_simulation_uses_dedicated_velocity_motor_effort_envelope(
    simulation_robot: ET.Element,
) -> None:
    """Do not regress to hardware torque limits for Gazebo's ideal velocity motors."""
    joints = {joint.attrib["name"]: joint for joint in simulation_robot.findall("joint")}
    emitted = {
        name: float(joints[name].find("limit").attrib["effort"])
        for name in UR10E_SIMULATOR_EFFORT_ENVELOPE
    }
    assert emitted == pytest.approx(UR10E_SIMULATOR_EFFORT_ENVELOPE)


def test_gazebo_plugin_uses_explicit_controller_configuration(
    simulation_robot: ET.Element,
) -> None:
    plugins = simulation_robot.findall("gazebo/plugin")
    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.attrib == {
        "filename": "libgz_ros2_control-system.so",
        "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
    }
    assert Path(plugin.findtext("parameters")) == CONTROLLER_CONFIG
    assert plugin.findtext("hold_joints") == "true"
    gains = plugin.findall("position_proportional_gain")
    assert len(gains) == 1
    # Applies only to the position-commanded gripper; arm and rail use velocity feedforward.
    assert float(gains[0].text) == pytest.approx(0.5)


def test_controller_configuration_partitions_commands_once() -> None:
    configuration = yaml.safe_load(CONTROLLER_CONFIG.read_text(encoding="utf-8"))
    manager = configuration["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 100
    assert manager["use_sim_time"] is True
    # This project keeps controller-manager's coupled post-limiter disabled and relies on the
    # upstream URDF hard bounds plus MoveIt's inner limits.
    assert manager["enforce_command_limits"] is False
    expected_types = {
        "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
        "arm_controller": "joint_trajectory_controller/JointTrajectoryController",
        "rail_controller": "joint_trajectory_controller/JointTrajectoryController",
        "gripper_controller": "joint_trajectory_controller/JointTrajectoryController",
    }
    for name, plugin_type in expected_types.items():
        assert manager[name]["type"] == plugin_type

    controller_joints = {
        name: configuration[name]["ros__parameters"]["joints"]
        for name in ("arm_controller", "rail_controller", "gripper_controller")
    }
    flattened = [joint for joints in controller_joints.values() for joint in joints]
    assert set(flattened) == COMMAND_JOINTS
    assert len(flattened) == len(set(flattened))
    assert controller_joints == {
        "arm_controller": [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
        "rail_controller": ["rail_joint"],
        "gripper_controller": ["left_finger_joint", "right_finger_joint"],
    }
    expected_command_interfaces = {
        "arm_controller": ["velocity"],
        "rail_controller": ["velocity"],
        "gripper_controller": ["position"],
    }
    for controller in controller_joints:
        parameters = configuration[controller]["ros__parameters"]
        assert parameters["command_interfaces"] == expected_command_interfaces[controller]
        assert parameters["state_interfaces"] == ["position", "velocity"]
        assert parameters["allow_partial_joints_goal"] is False

    for controller in ("arm_controller", "rail_controller"):
        gains = configuration[controller]["ros__parameters"]["gains"]
        assert set(gains) == set(controller_joints[controller])
        for gain in gains.values():
            assert gain == {
                "p": pytest.approx(10.0),
                "i": pytest.approx(0.0),
                "d": pytest.approx(0.0),
                "i_clamp": pytest.approx(0.0),
                "ff_velocity_scale": pytest.approx(1.0),
            }
    assert "gains" not in configuration["gripper_controller"]["ros__parameters"]

    gripper_constraints = configuration["gripper_controller"]["ros__parameters"]["constraints"]
    for finger in ("left_finger_joint", "right_finger_joint"):
        assert gripper_constraints[finger] == {
            "trajectory": pytest.approx(0.01),
            "goal": pytest.approx(0.003),
        }


def test_world_contains_required_runtime_systems_and_floor() -> None:
    root = ET.parse(WORLD_FILE).getroot()
    world = root.find("world")
    assert root.attrib["version"] == "1.9"
    assert world.attrib["name"] == "restocking"
    plugins = {plugin.attrib["name"] for plugin in world.findall("plugin")}
    assert plugins == {
        "gz::sim::systems::Physics",
        "gz::sim::systems::UserCommands",
        "gz::sim::systems::SceneBroadcaster",
        # Without Sensors the world renders nothing, so the cameras publish no topics; the failure
        # looks like a missing bridge.
        "gz::sim::systems::Sensors",
        "restocker_gazebo::AttachmentSystem",
    }
    # Without this system the RGB-D cameras load but never render, so the world starts cleanly
    # and publishes no images.
    sensors = world.find("plugin[@name='gz::sim::systems::Sensors']")
    assert sensors.attrib["filename"] == "gz-sim-sensors-system"
    assert sensors.findtext("render_engine") == "ogre2"
    attachment = world.find("plugin[@name='restocker_gazebo::AttachmentSystem']")
    assert attachment.attrib["filename"] == "restocker_attachment_system"
    assert float(world.findtext("physics/max_step_size")) == pytest.approx(0.001)
    # Every product is a cylinder, and DART's default narrowphase is ODE's, which has no
    # cylinder/cylinder collider: products interpenetrated while colliding with every box.
    # `test_product_contact_runtime` measures this against a running simulator; this checks the
    # declaration, because that test carries the `renderer` label and does not run under
    # `nix flake check`.
    assert world.findtext("physics/dart/collision_detector") in {"bullet", "fcl"}
    assert world.find("light[@name='workcell_sun']") is not None
    floor = world.find("model[@name='floor']")
    assert floor.findtext("static") == "true"
    assert floor.find("link/collision/geometry/box/size").text == "8 5 0.1"


def _expand_product(product: dict, label_texture: str = "") -> ET.Element:
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    geometries = {item["geometry_key"]: item for item in catalog["geometries"]}
    shape = geometries[product["geometry_key"]]["shape"]
    rgba = " ".join(str(component) for component in product["rgba"])
    expanded = subprocess.run(
        [
            "xacro",
            str(PRODUCT_XACRO),
            f"radius:={shape['radius_m']}",
            f"height:={shape['height_m']}",
            f"mass:={product['mass_kg']}",
            f"rgba:={rgba}",
            f"friction:={product['friction']}",
            f"label_texture:={label_texture}",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(expanded)
        urdf.flush()
        subprocess.run(["check_urdf", urdf.name], check=True, capture_output=True, text=True)
    return ET.fromstring(expanded)


def test_product_scenario_has_unique_backend_identities_and_valid_metadata() -> None:
    scenario = yaml.safe_load(PRODUCT_SCENARIO.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    assert scenario["schema_version"] == 1
    assert scenario["pose_topic"] == "/world/restocking/pose/info"
    assert scenario["frame_id"] == "world"
    assert scenario["backend"] == {
        "name": "gazebo_ground_truth",
        "version": "harmonic_pose_v1",
    }
    covariance = scenario["pose_covariance_diagonal"]
    assert len(covariance) == 6
    assert all(float(value) > 0.0 for value in covariance)

    products = scenario["products"]
    geometries = {item["geometry_key"]: item for item in catalog["geometries"]}
    assert catalog["schema_version"] == 1
    assert len(geometries) == len(catalog["geometries"])
    assert {item["product_class"] for item in products} == {
        "can",
        "small_bottle",
        "large_bottle",
    }
    assert len({item["model_name"] for item in products}) == len(products)
    assert len({item["source_object_id"] for item in products}) == len(products)
    assert all(item["source_object_id"].startswith("sim:") for item in products)
    assert all(len(item["spawn_pose"]) == 6 for item in products)
    assert all("radius_m" not in item and "height_m" not in item for item in products)
    # The baseline stocks a subset of the catalog (the catalog has a second can it does not
    # spawn). The check that no catalogued geometry is dead weight is kept below, across every
    # scenario this package ships.
    assert {item["geometry_key"] for item in products} <= set(geometries)
    for product in products:
        geometry = geometries[product["geometry_key"]]
        assert geometry["product_class"] == product["product_class"]
        assert geometry["sku"] == product["sku"]
        assert geometry["class_fallback"] is True
        assert geometry["shape"]["type"] == "cylinder"
        assert float(geometry["shape"]["radius_m"]) > 0.0
        assert float(geometry["shape"]["height_m"]) > 0.0


@pytest.mark.parametrize("product_index", [0, 1, 2])
def test_parameterized_product_has_declared_geometry_and_physics(product_index: int) -> None:
    product = yaml.safe_load(PRODUCT_SCENARIO.read_text(encoding="utf-8"))["products"][
        product_index
    ]
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    geometry = next(
        item for item in catalog["geometries"] if item["geometry_key"] == product["geometry_key"]
    )
    robot = _expand_product(product)
    links = robot.findall("link")
    assert [link.attrib["name"] for link in links] == ["product_body"]
    link = links[0]
    visual = link.find("visual/geometry/cylinder").attrib
    collision = link.find("collision/geometry/cylinder").attrib
    expected = {
        "radius": str(geometry["shape"]["radius_m"]),
        "length": str(geometry["shape"]["height_m"]),
    }
    assert visual == expected
    assert collision == expected

    inertial = link.find("inertial")
    assert float(inertial.find("mass").attrib["value"]) == pytest.approx(product["mass_kg"])
    inertia = inertial.find("inertia").attrib
    assert all(float(inertia[name]) > 0.0 for name in ("ixx", "iyy", "izz"))
    assert all(float(inertia[name]) == 0.0 for name in ("ixy", "ixz", "iyz"))

    gazebo = robot.find("gazebo[@reference='product_body']")
    assert float(gazebo.findtext("mu1")) == pytest.approx(product["friction"])
    assert float(gazebo.findtext("mu2")) == pytest.approx(product["friction"])


def test_every_catalogued_geometry_is_stocked_by_some_shipped_scenario() -> None:
    """A catalogued shape nothing can spawn is a dimension nobody ever measured."""
    # Stated over every scenario in the package rather than the baseline alone, since that is the
    # set a catalogue entry can earn its place in.
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    catalogued = {item["geometry_key"] for item in catalog["geometries"]}
    stocked = set()
    for path in sorted(SCENARIO_DIR.glob("*_products.yaml")):
        scenario = yaml.safe_load(path.read_text(encoding="utf-8"))
        keys = {product["geometry_key"] for product in scenario["products"]}
        assert keys <= catalogued, f"{path.name} stocks {sorted(keys - catalogued)}"
        stocked |= keys
    assert stocked == catalogued, f"no scenario stocks {sorted(catalogued - stocked)}"


def test_same_shape_scenario_stocks_two_skus_a_camera_has_to_separate() -> None:
    """Two products of one shape and different labels is the case vision work needs to exist."""
    scenario = yaml.safe_load(SAME_SHAPE_SCENARIO.read_text(encoding="utf-8"))
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    geometries = {item["geometry_key"]: item for item in catalog["geometries"]}

    products = scenario["products"]
    assert len(products) >= 2
    shapes = {
        (
            geometries[product["geometry_key"]]["shape"]["radius_m"],
            geometries[product["geometry_key"]]["shape"]["height_m"],
        )
        for product in products
    }
    assert len(shapes) == 1, f"the products differ in size, so a ruler identifies them: {shapes}"
    assert len({product["sku"] for product in products}) == len(products)
    # A product with no label would be identified by the absence of one.
    labels = [product["label_texture"] for product in products]
    assert len(set(labels)) == len(labels)
    for label in labels:
        assert (TEXTURE_DIR / label).is_file(), label


def test_label_texture_reaches_the_visual_gazebo_merges_extensions_into() -> None:
    """A label that lands on a differently named visual is silently dropped by sdformat."""
    # sdformat only merges a `<gazebo reference>` visual extension into a visual whose generated
    # SDF name contains `<link>_visual`, so the name matters. This asserts the name and the
    # albedo map that rides on it; getting either wrong costs nothing at launch and produces an
    # untextured can.
    product = yaml.safe_load(SAME_SHAPE_SCENARIO.read_text(encoding="utf-8"))["products"][0]
    texture = str(TEXTURE_DIR / product["label_texture"])
    robot = _expand_product(product, label_texture=texture)
    assert robot.find("link/visual").attrib["name"] == "product_body_visual"
    gazebo = robot.find("gazebo[@reference='product_body']")
    assert gazebo.findtext("visual/material/pbr/metal/albedo_map") == texture
    assert float(gazebo.findtext("visual/material/pbr/metal/metalness")) == 0.0

    bare = _expand_product(product)
    assert bare.find("gazebo[@reference='product_body']/visual") is None
