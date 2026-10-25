# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static tests for the Gazebo model, control interfaces, and controller partition."""

import math
import os
from pathlib import Path
import shutil
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
        # Every joint exports position and velocity; the two finger joints also export effort,
        # the one their controller claims (spec M10 §11 D10 item 5). controllers.yaml decides
        # which one each controller claims, and that partition is asserted separately below.
        expected_interfaces = (
            ["position", "velocity", "effort"] if "finger" in name else ["position", "velocity"]
        )
        assert [
            item.attrib["name"] for item in hardware_joint.findall("command_interface")
        ] == expected_interfaces
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
    # Spec M10 §11 D10 item 5: the effort-driven fingers need their force command held for one
    # physics step, not ten, so the controller loop runs at the world's <max_step_size>.
    assert manager["update_rate"] == 1000
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
        "gripper_controller": ["effort"],
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
    # Spec M10 §11 D10 item 5: the gripper's force-mode position controller, every number derived
    # (p from effort_n and the interference the pad compression eats out of it, d from the joint
    # velocity limit the force clamp no longer has, u_clamp from effort_n itself). The arm and
    # rail still declare no gains beyond their feedforward set, asserted above.
    finger_gains = configuration["gripper_controller"]["ros__parameters"]["gains"]
    assert set(finger_gains) == {"left_finger_joint", "right_finger_joint"}
    grip_geometry = _finger_geometry()
    effort_n = float(grip_geometry["joint"]["effort_n"])
    row_stiffness = float(grip_geometry["pad"]["row_stiffness_n_per_m"])
    interference = float(
        yaml.safe_load(GRIPPER_GEOMETRY.read_text(encoding="utf-8"))["attachment"][
            "hold_interference_m"
        ]
    )
    velocity_limit = float(grip_geometry["joint"]["velocity_mps"])
    joint_damping = float(grip_geometry["joint"]["damping_n_s_per_m"])
    # Spec M10 section 11 D10 item 5: the loop must saturate at effort_n for every stall error,
    # i.e. effort_n / p <= interference - effort_n / row_stiffness, and that bound is strictest
    # at the smallest interference the design has used (1 mm), so that is what pins the gain.
    saturation_error = 0.001 - effort_n / row_stiffness
    expected_p = effort_n / saturation_error
    expected_d = effort_n / velocity_limit - joint_damping
    for gains in finger_gains.values():
        # The shipped numbers are rounded to a whole N/m and N*s/m; the derivation is exact.
        assert gains["p"] == pytest.approx(expected_p, abs=1.0)
        assert gains["d"] == pytest.approx(expected_d, abs=0.5)
        # And the clamp must engage before the jaw can stall: the saturation error has to sit
        # inside the smallest stall error the shipped interference admits.
        assert effort_n / gains["p"] < interference
        assert gains["i"] == pytest.approx(0.0)
        assert gains["u_clamp_max"] == pytest.approx(effort_n)
        assert gains["u_clamp_min"] == pytest.approx(-effort_n)

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
        # The pad rubber (spec M10 section 11 D10 item 3): the engine does not act on the joints'
        # spring_stiffness/spring_reference/damping, so this system commands the force. Without it
        # the pad rows are free sliders and SC-002's compliance is absent.
        "restocker_gazebo::PadSpringSystem",
    }
    # Without this system the RGB-D cameras load but never render, so the world starts cleanly
    # and publishes no images.
    sensors = world.find("plugin[@name='gz::sim::systems::Sensors']")
    assert sensors.attrib["filename"] == "gz-sim-sensors-system"
    assert sensors.findtext("render_engine") == "ogre2"
    attachment = world.find("plugin[@name='restocker_gazebo::AttachmentSystem']")
    assert attachment.attrib["filename"] == "restocker_attachment_system"
    pad_spring = world.find("plugin[@name='restocker_gazebo::PadSpringSystem']")
    assert pad_spring.attrib["filename"] == "restocker_pad_spring_system"
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


def _expand_product_text(product: dict, label_texture: str = "", visual_mesh: str = "") -> str:
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    geometries = {item["geometry_key"]: item for item in catalog["geometries"]}
    shape = geometries[product["geometry_key"]]["shape"]
    rgba = " ".join(str(component) for component in product["rgba"])
    if shape["type"] == "box":
        # A box has no radius: the xacro takes its three sizes and ignores radius and height.
        size_arguments = ["box_size:=" + " ".join(str(v) for v in shape["size_xyz_m"])]
    else:
        size_arguments = [f"radius:={shape['radius_m']}", f"height:={shape['height_m']}"]
    expanded = subprocess.run(
        [
            "xacro",
            str(PRODUCT_XACRO),
            *size_arguments,
            f"mass:={product['mass_kg']}",
            f"rgba:={rgba}",
            f"friction:={product['friction']}",
            f"label_texture:={label_texture}",
            f"visual_mesh:={visual_mesh}",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(expanded)
        urdf.flush()
        subprocess.run(["check_urdf", urdf.name], check=True, capture_output=True, text=True)
    return expanded


def _expand_product(product: dict, label_texture: str = "", visual_mesh: str = "") -> ET.Element:
    return ET.fromstring(
        _expand_product_text(product, label_texture=label_texture, visual_mesh=visual_mesh)
    )


def _convert_product_to_sdf(product: dict, label_texture: str = "") -> ET.Element:
    """Run the URDF->SDF conversion sdformat performs inside `ros_gz_sim create` at launch."""
    # The `<gazebo reference>` extension only means something after this step: sdformat merges it
    # into the element whose generated name matches, and under any other name it is discarded with
    # no diagnostic. Asserting the URDF therefore always looks green, which is how the catalogued
    # friction went unmeasured for the life of the file (Card 024).
    assert shutil.which("gz") is not None, "the gz CLI is what performs this conversion"
    expanded = _expand_product_text(product, label_texture=label_texture)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(expanded)
        urdf.flush()
        converted = subprocess.run(
            ["gz", "sdf", "-p", urdf.name],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    return ET.fromstring(converted)


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


def test_product_collision_carries_the_name_sdformat_merges_extensions_into() -> None:
    """The collision name is the whole mechanism (Card 024), so pin it at the source."""
    # sdformat looks for `<link>_collision` when it merges `mu1`/`mu2`. Any other collision name
    # and the extension is discarded with no diagnostic — the state of the file from the day it
    # was written until 2026-09-29, during which the catalogued friction never reached Gazebo.
    root = ET.parse(PRODUCT_XACRO).getroot()
    collisions = root.findall("link[@name='product_body']/collision")
    assert [collision.attrib["name"] for collision in collisions] == ["product_body_collision"]


@pytest.mark.parametrize("product_index", [0, 1, 2])
def test_catalogued_friction_reaches_the_converted_product_collision(product_index: int) -> None:
    """Card 024 SC-001: assert the SDF the engine is given, not the URDF that looks right."""
    # The URDF assertion above passes either way; this one is red while the collision is named
    # `product_collision`, because `surface` is then absent from the converted document.
    product = yaml.safe_load(PRODUCT_SCENARIO.read_text(encoding="utf-8"))["products"][
        product_index
    ]
    sdf = _convert_product_to_sdf(product)
    collisions = sdf.findall(".//collision")
    assert len(collisions) == 1
    surface = collisions[0].find("surface")
    assert surface is not None, "sdformat dropped the <gazebo reference> friction extension"
    friction = surface.find("friction/ode")
    assert friction is not None
    # DART takes the smaller of the two bodies' coefficients, so this value is what every product
    # contact actually uses (spec 11.2 F5) and must be the catalogued one.
    assert float(friction.findtext("mu")) == pytest.approx(product["friction"])
    assert float(friction.findtext("mu2")) == pytest.approx(product["friction"])
    # kp/kd reach the document too and are then ignored by the DART backend (spec 11.2 F7).
    # Asserting they survive proves the whole extension merged; asserting a *value* for them would
    # claim a stiffness tuning surface that does not exist.
    contact = surface.find("contact/ode")
    assert contact is not None
    assert float(contact.findtext("kp")) > 0.0
    assert float(contact.findtext("kd")) > 0.0


def test_label_and_friction_survive_the_conversion_together() -> None:
    """The material and the friction are two extensions of one `<gazebo reference>` block."""
    # Proves the collision rename did not cost the label path: the visual still merges its albedo
    # map while the collision merges its friction, in the same converted document.
    product = yaml.safe_load(SAME_SHAPE_SCENARIO.read_text(encoding="utf-8"))["products"][0]
    texture = str(TEXTURE_DIR / product["label_texture"])
    sdf = _convert_product_to_sdf(product, label_texture=texture)
    collision = sdf.find(".//collision")
    assert collision is not None
    assert collision.findtext("surface/friction/ode/mu") is not None
    assert sdf.findtext(".//visual/material/pbr/metal/albedo_map") == texture


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


GRIPPER_GEOMETRY = DESCRIPTION_SOURCE / "config" / "gripper_geometry.yaml"
TASK_EXECUTOR_SOURCE = DESCRIPTION_SOURCE.parent / "restocker_task_executor"
COORDINATOR_CONFIG = TASK_EXECUTOR_SOURCE / "config" / "restock_action_coordinator.yaml"


def _finger_geometry() -> dict:
    return yaml.safe_load(GRIPPER_GEOMETRY.read_text(encoding="utf-8"))["finger"]


def _converted_simulation_robot(simulation_robot: ET.Element) -> ET.Element:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf") as urdf:
        urdf.write(ET.tostring(simulation_robot, encoding="unicode"))
        urdf.flush()
        return ET.fromstring(
            subprocess.run(
                ["gz", "sdf", "-p", urdf.name], check=True, capture_output=True, text=True
            ).stdout
        )


def test_compliant_pad_rows_reach_the_converted_robot_sdf(simulation_robot: ET.Element) -> None:
    """
    Spec M10 §11 D9 + D10: the pad is pad_rows rows as their own spring-mounted links.

    Pinned on the converted SDF because that is what Gazebo simulates: a row, its spring, or the
    finger body's shortened collision being lost in the URDF-to-SDF reduction would pass any
    URDF-level assertion, and the union of body + rows is the whole geometric contract the
    attach predicates and MoveIt still see.
    """
    finger = _finger_geometry()
    rows = finger["pad_rows"]
    height, thickness, length = finger["size_xyz_m"]
    center = finger["center_xyz_m"]
    pad = finger["pad"]
    row_height = height / rows
    # A row is as deep as it is high: the 3.3 mm silicone layer sets the pad's compliance (the
    # joint's spring and travel), not the collision box, whose back edges a 3 mm press would reach.
    row_depth = row_height
    converted = _converted_simulation_robot(simulation_robot)

    for side, inner_sign in (("left", -1.0), ("right", 1.0)):
        body = converted.find(f".//link[@name='{side}_finger']")
        assert body is not None
        body_collisions = body.findall("collision")
        assert len(body_collisions) == 1
        size = [float(v) for v in body_collisions[0].findtext("geometry/box/size").split()]
        pose = [float(v) for v in body_collisions[0].findtext("pose").split()]
        # The finger body keeps the box minus the row depth, pushed away from the product.
        assert size == pytest.approx([height, thickness - row_depth, length])
        assert pose[1] == pytest.approx(-inner_sign * 0.5 * row_depth)
        assert pose[0] == pytest.approx(center[0])
        assert pose[2] == pytest.approx(center[2])

        covered = [(pose[1] - 0.5 * size[1], pose[1] + 0.5 * size[1])]
        for index in range(rows):
            link = converted.find(f".//link[@name='{side}_finger_pad_row_{index}']")
            assert link is not None
            collisions = link.findall("collision")
            assert len(collisions) == 1
            row_size = [float(v) for v in collisions[0].findtext("geometry/box/size").split()]
            row_pose = [float(v) for v in collisions[0].findtext("pose").split()]
            assert row_size == pytest.approx([row_height, row_depth, length])
            assert row_pose[1] == pytest.approx(inner_sign * (0.5 * thickness - 0.5 * row_depth))
            assert row_pose[2] == pytest.approx(center[2])
            # The rows tile the pad height exactly where D9 put them.
            assert row_pose[0] - 0.5 * row_height == pytest.approx(
                center[0] - 0.5 * height + index * row_height
            )
            covered.append((row_pose[1] - 0.5 * row_size[1], row_pose[1] + 0.5 * row_size[1]))
            assert float(link.findtext("inertial/mass")) == pytest.approx(
                float(pad["row_mass_kg"])
            )

            joint = converted.find(f".//joint[@name='{side}_finger_pad_row_{index}_joint']")
            assert joint is not None
            assert joint.findtext("parent") == f"{side}_finger"
            axis = [float(v) for v in joint.findtext("axis/xyz").split()]
            # The pad compresses away from the product: -inner_sign along the jaw axis.
            assert axis == pytest.approx([0.0, -inner_sign, 0.0])
            dynamics = joint.find("axis/dynamics")
            assert dynamics is not None
            assert float(dynamics.findtext("spring_stiffness")) == pytest.approx(
                float(pad["row_stiffness_n_per_m"])
            )
            assert float(dynamics.findtext("spring_reference")) == pytest.approx(0.0)
            assert float(dynamics.findtext("damping")) == pytest.approx(
                float(pad["row_damping_n_s_per_m"])
            )
            lower = float(joint.findtext("axis/limit/lower"))
            upper = float(joint.findtext("axis/limit/upper"))
            # Rest sits strictly inside the limits, so DART never starts by enforcing one.
            assert lower < 0.0 < upper
            # The pad joints are passive and carry the spring force restocker_pad_spring_system
            # commands (spec M10 section 11 D10 item 3). Their effort has to clear the spring's own
            # bound: k*travel, plus the damping at the fastest the pad can move — a 40 N step
            # drives it at v = effort/sqrt(k*m) = 1.71 m/s, so 222 + 19.7 = 241.7 N — not the
            # drive's effort_n. DART clips a force command to the effort limit (section 11.2 F14).
            travel = min(abs(lower), upper)
            stiffness = float(pad["row_stiffness_n_per_m"])
            peak_speed = (
                float(finger["joint"]["effort_n"]) / (stiffness * float(pad["row_mass_kg"])) ** 0.5
            )
            spring_bound = stiffness * travel + float(pad["row_damping_n_s_per_m"]) * peak_speed
            pad_effort = float(pad["effort_n"])
            assert pad_effort > spring_bound
            assert float(joint.findtext("axis/limit/effort")) == pytest.approx(pad_effort)
            # Spec M10 section 11 D10 item 3: a bonded pad has no speed limit of its own, so the
            # pad joints are unbounded (-1, which sdformat maps to infinity; 0 would freeze them)
            # rather than inheriting the jaw's 0.12 m/s — 14x below what that same step needs.
            assert float(joint.findtext("axis/limit/velocity")) == pytest.approx(-1.0)

        # Body plus rows cover the pad thickness exactly once: the union is the old finger box.
        # The four rows share one y interval (they tile x, not y), so the intervals are merged
        # rather than summed.
        intervals = sorted(covered)
        merged = []
        for low, high in intervals:
            if merged and low <= merged[-1][1] + 1e-12:
                merged[-1][1] = max(merged[-1][1], high)
            else:
                merged.append([low, high])
        assert len(merged) == 1
        assert merged[0][0] == pytest.approx(-0.5 * thickness)
        assert merged[0][1] == pytest.approx(0.5 * thickness)
        assert merged[0][1] - merged[0][0] == pytest.approx(thickness)


def test_grip_force_is_the_effort_limit_of_the_finger_joint(
    simulation_robot: ET.Element,
) -> None:
    """Spec M10 §11 D10: effort_n is both the controller's clamp and the joint's force limit."""
    finger = _finger_geometry()
    joints = {joint.attrib["name"]: joint for joint in simulation_robot.findall("joint")}
    for side in ("left", "right"):
        limit = joints[f"{side}_finger_joint"].find("limit")
        assert limit is not None
        assert float(limit.attrib["effort"]) == pytest.approx(float(finger["joint"]["effort_n"]))


def test_pad_rows_is_the_smallest_count_giving_every_product_three_whole_rows() -> None:
    """Spec M10 §11 D9's rule, recomputed from the catalogue and the production grasp heights."""
    finger = _finger_geometry()
    pad_height = finger["size_xyz_m"][0]
    pad_center = finger["center_xyz_m"][0]
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))["geometries"]
    parameters = yaml.safe_load(COORDINATOR_CONFIG.read_text(encoding="utf-8"))[
        "restock_action_coordinator"
    ]["ros__parameters"]
    minimum_center = parameters["grasp.minimum_center_height_above_base_m"]
    maximum_top = parameters["grasp.maximum_top_above_center_m"]

    def whole_rows(rows: int, product_height: float) -> int:
        # grasp_candidates.cpp: mid-height, raised only as far as the two clearances require.
        half = 0.5 * product_height
        above_base = half + max(0.0, minimum_center - half, half - maximum_top)
        # In a side grasp grasp +x is the product axis, up; the pad is centred on pad_center.
        low, high = -above_base, product_height - above_base
        row = pad_height / rows
        edges = [pad_center - 0.5 * pad_height + index * row for index in range(rows)]
        return sum(1 for edge in edges if edge >= low - 1e-12 and edge + row <= high + 1e-12)

    # Boxes are not grasped (CMB-119 stage 1), so the pad rows are sized on the cylinders only.
    heights = [
        entry["shape"]["height_m"] for entry in catalog if entry["shape"]["type"] == "cylinder"
    ]
    derived = next(
        rows for rows in range(1, 64) if min(whole_rows(rows, height) for height in heights) >= 3
    )
    assert finger["pad_rows"] == derived


GENERATED_SCENARIO = SCENARIO_DIR / "generated_catalogue_products.yaml"
MESH_DIR = GAZEBO_SOURCE / "meshes"


def test_contoured_bottle_is_a_visual_mesh_over_a_cylinder_collision() -> None:
    """The contour is cosmetic: physics, grasping and the catalogue loaders see a cylinder."""
    scenario = yaml.safe_load(GENERATED_SCENARIO.read_text(encoding="utf-8"))
    product = next(item for item in scenario["products"] if item.get("visual_mesh"))
    mesh = str(MESH_DIR / product["visual_mesh"])
    robot = _expand_product(
        product, label_texture=str(TEXTURE_DIR / product["label_texture"]), visual_mesh=mesh
    )
    visual = robot.find(".//link/visual")
    assert visual.find("geometry/mesh").get("filename") == f"file://{mesh}"
    assert visual.find("geometry/cylinder") is None
    collision = robot.find(".//link/collision")
    assert collision.find("geometry/cylinder") is not None
    assert collision.find("geometry/mesh") is None


def test_plain_product_still_draws_a_cylinder_without_a_mesh() -> None:
    product = yaml.safe_load(SAME_SHAPE_SCENARIO.read_text(encoding="utf-8"))["products"][0]
    visual = _expand_product(product).find(".//link/visual")
    assert visual.find("geometry/cylinder") is not None
    assert visual.find("geometry/mesh") is None


def test_contour_mesh_fits_inside_its_collision_cylinder() -> None:
    """A visual wider or taller than the collision would hover above or clip into its support."""
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    scenario = yaml.safe_load(GENERATED_SCENARIO.read_text(encoding="utf-8"))
    geometries = {item["geometry_key"]: item["shape"] for item in catalog["geometries"]}
    for product in scenario["products"]:
        if not product.get("visual_mesh"):
            continue
        shape = geometries[product["geometry_key"]]
        points = [
            tuple(float(v) for v in line.split()[1:4])
            for line in (MESH_DIR / product["visual_mesh"]).read_text().splitlines()
            if line.startswith("v ")
        ]
        if shape["type"] == "box":
            # A box mesh is the collision box itself, corner for corner.
            for axis, size in enumerate(shape["size_xyz_m"]):
                assert max(abs(point[axis]) for point in points) == pytest.approx(
                    size / 2, abs=1e-6
                )
            continue
        assert max(math.hypot(x, y) for x, y, _ in points) <= shape["radius_m"] + 1e-3
        assert max(abs(z) for _, _, z in points) <= shape["height_m"] / 2 + 1e-3


def _box_product() -> dict:
    scenario = yaml.safe_load(GENERATED_SCENARIO.read_text(encoding="utf-8"))
    return next(item for item in scenario["products"] if item["product_class"] == "box")


def test_box_product_has_a_box_collision_box_inertia_and_its_atlas_mesh() -> None:
    product = _box_product()
    catalog = yaml.safe_load(PRODUCT_CATALOG.read_text(encoding="utf-8"))
    shape = next(
        item["shape"]
        for item in catalog["geometries"]
        if item["geometry_key"] == product["geometry_key"]
    )
    mesh = str(MESH_DIR / product["visual_mesh"])
    robot = _expand_product(product, visual_mesh=mesh)
    size = " ".join(str(v) for v in shape["size_xyz_m"])
    assert robot.find(".//link/collision/geometry/box").get("size") == size
    assert robot.find(".//link/collision/geometry/cylinder") is None
    assert robot.find(".//link/visual/geometry/mesh").get("filename") == f"file://{mesh}"
    # Without a mesh the visual is the same box.
    plain = _expand_product(product)
    assert plain.find(".//link/visual/geometry/box").get("size") == size
    inertial = robot.find(".//link/inertial")
    x, y, z = shape["size_xyz_m"]
    mass = float(inertial.find("mass").attrib["value"])
    inertia = inertial.find("inertia").attrib
    assert float(inertia["ixx"]) == pytest.approx(mass * (y * y + z * z) / 12.0)
    assert float(inertia["izz"]) == pytest.approx(mass * (x * x + y * y) / 12.0)


def test_box_product_keeps_its_friction_and_texture_through_the_sdf_conversion() -> None:
    product = _box_product()
    texture = str(TEXTURE_DIR / product["label_texture"])
    sdf = _convert_product_to_sdf(product, label_texture=texture)
    assert sdf.find(".//collision/geometry/box") is not None
    friction = sdf.find(".//collision/surface/friction/ode/mu")
    assert float(friction.text) == pytest.approx(product["friction"])
    assert texture in ET.tostring(sdf, encoding="unicode")
