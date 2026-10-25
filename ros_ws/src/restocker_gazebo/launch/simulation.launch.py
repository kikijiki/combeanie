# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Launch Gazebo Harmonic, the robot model, and its ros2_control controllers."""

from functools import cache
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from restocker_gazebo.label_textures import install_texture, load_texture_rotations
from restocker_gazebo.scenario_config import (
    load_product_catalog,
    load_scenario,
    resolve_product_geometry,
)
from restocker_gazebo.scenario_random import (
    FIXED_SCENARIO_SEED,
    generate_scenario_from_files,
    parse_seed,
    write_generated_scenario,
)
from restocker_gazebo.world_bundle import BundledModel, bundle_models, load_materials


def generated_scenario_directory() -> Path:
    """Return the directory seeded scenarios are materialized into."""
    # The ground-truth adapter and the in-process Gazebo attachment plugin take a scenario path,
    # not a scenario, so the generated document must exist as a file. The path is stable per seed
    # so a failing run leaves the file it used, and per-user so operators on one machine do not
    # collide in a shared temporary directory.
    override = os.environ.get("RESTOCKER_GENERATED_SCENARIO_DIR")
    if override:
        return Path(override)
    return Path(tempfile.gettempdir()) / f"restocker-scenarios-{os.getuid()}"


def _resolve_scenario(context):
    """Replace the scenario path with a seeded draw before anything else reads it."""
    # Runs before the environment variables and the Gazebo include: the attachment plugin reads
    # RESTOCKER_SCENARIO_CONFIG inside the simulator process at startup, so rewriting the path
    # later would leave it validating a different scenario than the one spawned.
    seed = parse_seed(LaunchConfiguration("scenario_seed").perform(context))
    if seed is None:
        return []
    scenario = generate_scenario_from_files(
        LaunchConfiguration("scenario_config").perform(context),
        LaunchConfiguration("product_catalog").perform(context),
        LaunchConfiguration("workcell_geometry").perform(context),
        LaunchConfiguration("gripper_geometry").perform(context),
        seed,
    )
    path = write_generated_scenario(scenario, generated_scenario_directory())
    return [
        LogInfo(
            msg=(
                f"scenario_seed={seed} generated a {len(scenario['products'])}-product scenario "
                f"at {path}; replay this exact run with scenario_seed:={seed}"
            )
        ),
        SetLaunchConfiguration("scenario_config", str(path)),
    ]


def _pose_parameters(pose: list[float]) -> dict[str, float]:
    """Map xyz/rpy scenario order to ros_gz_sim create parameter names."""
    return dict(zip(("x", "y", "z", "R", "P", "Y"), pose, strict=True))


@cache
def _texture_rotations(manifest: str) -> dict[str, int]:
    """The label manifest's rotations, read once however many products ask."""
    return load_texture_rotations(manifest)


def _product_xacro_arguments(product, catalog) -> dict[str, str]:
    """The product.urdf.xacro arguments of one scenario product, as the text xacro receives."""
    geometry = resolve_product_geometry(product, catalog)
    shape = geometry["shape"]
    box = shape["type"] == "box"
    package_share = Path(get_package_share_directory("restocker_gazebo"))
    # A scenario names a label by filename only. Resolving it here keeps the scenario free of
    # absolute paths and the URDF free of package layout; Gazebo would otherwise resolve an albedo
    # map relative to its own working directory.
    texture_root = package_share / "materials" / "textures"
    mesh_root = package_share / "meshes"
    # Strips are stored upright; the half-turn a cylinder primitive's UVs need is applied by
    # install_texture, driven by the manifest (see label_textures.py).
    texture_rotations = _texture_rotations(
        str(package_share / "config" / "generated_product_labels.yaml")
    )
    label = product.get("label_texture", "")
    label_texture = ""
    if label:
        # Gazebo does not complain about an albedo map it cannot open: it draws the flat base
        # colour, which looks like a product never given a label and which a vision
        # measurement would silently absorb.
        if not (texture_root / label).is_file():
            raise RuntimeError(f"label texture is not installed: {label}")
        label_texture = str(install_texture(texture_root / label, texture_rotations.get(label, 0)))
    mesh = product.get("visual_mesh", "")
    visual_mesh = ""
    if mesh:
        if not (mesh_root / mesh).is_file():
            raise RuntimeError(f"visual mesh is not installed: {mesh}")
        visual_mesh = str(mesh_root / mesh)
    return {
        "radius": str(shape.get("radius_m", 0.033)),
        "height": str(shape.get("height_m", shape["size_xyz_m"][2] if box else 0.122)),
        "box_size": " ".join(str(v) for v in shape["size_xyz_m"]) if box else "",
        "mass": str(product["mass_kg"]),
        "friction": str(product["friction"]),
        "static": "true" if product.get("static", False) else "false",
        "rgba": " ".join(str(value) for value in product["rgba"]),
        "label_texture": label_texture,
        "visual_mesh": visual_mesh,
    }


def _bundle_world(context):
    """Replace the world with a copy that already holds the workcell and every product."""
    # Runs after _resolve_scenario (the scenario path is final) and before the Gazebo include
    # reads `world`. The workcell and products are then not spawned by create processes.
    if LaunchConfiguration("bundle_scenario").perform(context).lower() != "true":
        return []
    if LaunchConfiguration("spawn_scenario").perform(context).lower() != "true":
        return []
    scenario = load_scenario(LaunchConfiguration("scenario_config").perform(context))
    catalog = load_product_catalog(LaunchConfiguration("product_catalog").perform(context))
    workcell_geometry = LaunchConfiguration("workcell_geometry").perform(context)
    world_path = Path(LaunchConfiguration("world").perform(context))
    description_share = Path(get_package_share_directory("restocker_description"))
    gazebo_share = Path(get_package_share_directory("restocker_gazebo"))
    models = [
        BundledModel(
            str(description_share / "urdf" / "workcell.urdf.xacro"),
            {"geometry_config": workcell_geometry},
            "workcell",
            [float(value) for value in scenario["workcell_pose"]],
        )
    ]
    product_xacro = str(gazebo_share / "urdf" / "product.urdf.xacro")
    for product in scenario["products"]:
        models.append(
            BundledModel(
                product_xacro,
                _product_xacro_arguments(product, catalog),
                product["model_name"],
                [float(value) for value in product["spawn_pose"]],
            )
        )
    materials_path = LaunchConfiguration("scene_materials").perform(context)
    bundled = bundle_models(
        world_path.read_text(encoding="utf-8").replace("@GAZEBO_SHARE@", str(gazebo_share)),
        models,
        materials=load_materials(materials_path) if materials_path else None,
    )
    target = generated_scenario_directory() / f"bundled_{os.getpid()}_{world_path.name}"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(bundled, encoding="utf-8")
    return [
        LogInfo(msg=f"bundled {len(models)} models into {target}"),
        SetLaunchConfiguration("world", str(target)),
    ]


def _scenario_actions(context):
    """Expand one validated workcell and the configured parameterized products."""
    scenario_config = LaunchConfiguration("scenario_config").perform(context)
    product_catalog = LaunchConfiguration("product_catalog").perform(context)
    workcell_geometry = LaunchConfiguration("workcell_geometry").perform(context)
    scenario = load_scenario(scenario_config)
    catalog = load_product_catalog(product_catalog)
    enabled = IfCondition(LaunchConfiguration("spawn_scenario"))
    bundled = LaunchConfiguration("bundle_scenario").perform(context).lower() == "true"
    xacro_executable = FindExecutable(name="xacro")
    workcell_xacro = str(
        Path(get_package_share_directory("restocker_description")) / "urdf" / "workcell.urdf.xacro"
    )
    product_xacro = str(
        Path(get_package_share_directory("restocker_gazebo")) / "urdf" / "product.urdf.xacro"
    )
    workcell_description = ParameterValue(
        Command(
            [
                xacro_executable,
                " ",
                workcell_xacro,
                " geometry_config:=",
                workcell_geometry,
            ]
        ),
        value_type=str,
    )
    workcell_pose = [float(value) for value in scenario["workcell_pose"]]
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="workcell_state_publisher",
            output="screen",
            parameters=[{"robot_description": workcell_description, "use_sim_time": True}],
            remappings=[("robot_description", "workcell/robot_description")],
            condition=enabled,
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_shelf",
            output="screen",
            arguments=[
                "--x",
                str(workcell_pose[0]),
                "--y",
                str(workcell_pose[1]),
                "--z",
                str(workcell_pose[2]),
                "--roll",
                str(workcell_pose[3]),
                "--pitch",
                str(workcell_pose[4]),
                "--yaw",
                str(workcell_pose[5]),
                "--frame-id",
                "world",
                "--child-frame-id",
                "shelf",
            ],
            condition=enabled,
        ),
    ]
    # The spawns are paced instead of started together: a twenty-plus-product scenario used to
    # join every create process to the test's ROS domain within a few seconds, and that
    # participant storm swallowed the clock bridge's initial discovery announcements. The
    # bridge's relay was created on time but delivered its first /clock message only about one
    # Fast DDS lease period later (~105 s measured), which is past the old 60 s startup bound
    # and sometimes past any bound (see wait_for_clock). Pacing keeps peak join churn at roughly
    # one process per interval without changing what gets spawned.
    creates = [
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_workcell",
            output="screen",
            parameters=[
                {
                    "world": "restocking",
                    "string": workcell_description,
                    "name": "workcell",
                    "allow_renaming": False,
                    **_pose_parameters(workcell_pose),
                }
            ],
            condition=enabled,
        ),
    ]

    for product in scenario["products"]:
        arguments = _product_xacro_arguments(product, catalog)
        product_description = ParameterValue(
            Command(
                [
                    xacro_executable,
                    " ",
                    product_xacro,
                    " radius:=",
                    arguments["radius"],
                    " height:=",
                    arguments["height"],
                    ' box_size:="',
                    arguments["box_size"],
                    '"',
                    " mass:=",
                    arguments["mass"],
                    " friction:=",
                    arguments["friction"],
                    " static:=",
                    arguments["static"],
                    ' rgba:="',
                    arguments["rgba"],
                    '"',
                    ' label_texture:="',
                    arguments["label_texture"],
                    '"',
                    ' visual_mesh:="',
                    arguments["visual_mesh"],
                    '"',
                ]
            ),
            value_type=str,
        )
        creates.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{product['model_name']}",
                output="screen",
                parameters=[
                    {
                        "world": "restocking",
                        "string": product_description,
                        "name": product["model_name"],
                        "allow_renaming": False,
                        **_pose_parameters([float(value) for value in product["spawn_pose"]]),
                    }
                ],
                condition=enabled,
            )
        )

    # A bundled world already holds the workcell and every product (see _bundle_world).
    if bundled:
        creates = []
    for index, create in enumerate(creates):
        actions.append(TimerAction(period=0.25 * (index + 1), actions=[create]))

    # Points at the topic the world state ingests, so ground truth is authoritative by default:
    # the camera producer swap is not yet sound, because the retreat after a release cannot be
    # planned while the product under the arm is unobservable. Moving this topic to
    # /perception/ground_truth/object_observations together with perception:=true performs the
    # swap and demotes ground truth to the evidence perception is measured against.
    #
    # Lane occupancy evidence is a different signal and remains ground truth.
    actions.append(
        Node(
            package="restocker_gazebo",
            executable="ground_truth_adapter",
            name="ground_truth_adapter",
            output="screen",
            parameters=[
                {
                    "scenario_config": scenario_config,
                    "workcell_geometry": workcell_geometry,
                    "product_catalog": product_catalog,
                    "output_topic": LaunchConfiguration("object_observation_topic"),
                    "use_sim_time": True,
                }
            ],
            condition=IfCondition(LaunchConfiguration("ground_truth")),
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Compose simulation startup with model-spawn and controller ordering."""
    gui = LaunchConfiguration("gui")
    controller_timeout = LaunchConfiguration("controller_timeout")
    world_file = LaunchConfiguration("world")
    default_scenario = PathJoinSubstitution(
        [FindPackageShare("restocker_gazebo"), "config", "baseline_products.yaml"]
    )
    default_product_catalog = PathJoinSubstitution(
        [FindPackageShare("restocker_description"), "config", "product_collision_catalog.yaml"]
    )
    default_workcell_geometry = PathJoinSubstitution(
        [FindPackageShare("restocker_description"), "config", "workcell_geometry.yaml"]
    )
    default_attachment_boundary = PathJoinSubstitution(
        [FindPackageShare("restocker_gazebo"), "config", "attachment_boundary.yaml"]
    )
    default_gripper_geometry = PathJoinSubstitution(
        [FindPackageShare("restocker_description"), "config", "gripper_geometry.yaml"]
    )

    controller_config = PathJoinSubstitution(
        [FindPackageShare("restocker_control"), "config", "controllers.yaml"]
    )
    simulation_xacro = PathJoinSubstitution(
        [FindPackageShare("restocker_gazebo"), "urdf", "restocker_sim.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                simulation_xacro,
                " controller_config:=",
                controller_config,
                # Gate the wrist sensor in the description, not at the bridge: `cameras:=false`
                # only stops the ROS bridge, while Gazebo keeps rendering anything the SDF
                # declares. Only dropping the sensor from the SDF stops the work, so this argument
                # reaches xacro rather than the bridge list below.
                " wrist_camera_sensor:=",
                LaunchConfiguration("wrist_camera"),
            ]
        ),
        value_type=str,
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        condition=IfCondition(gui),
        launch_arguments={
            "gz_args": ["-r -v 2 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        condition=UnlessCondition(gui),
        launch_arguments={
            "gz_args": ["-r -s -v 2 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    world_to_rail = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_rail_base",
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "rail_base",
        ],
    )
    # UDP-only for this participant alone: Fast DDS shared-memory port files are not
    # domain-scoped, and concurrent test domains were measured failing to open the same
    # fastrtps_portNNNN (56 init failures in one wave, including on this bridge — both ends of
    # the /clock hop). Scoping the preset here keeps that collision away from the clock path
    # while every other participant — camera bridges first — keeps shared memory for the
    # megapixel image topics: lease-wide UDP was tried and the first wave running it produced
    # jam-class controller aborts no shared-memory wave had shown (see domain_isolation).
    clock_bridge = GroupAction(
        [
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                output="screen",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            ),
        ]
    )
    # The RGB-D sensors publish on Gazebo transport only; this puts them on ROS. Same mechanism as
    # clock_bridge above: one parameter_bridge, `[` for Gazebo-to-ROS only. Both cameras share one
    # process because a second parameter_bridge would carry its own Gazebo transport thread pool.
    #
    # Neither point cloud is bridged: each is derivable from the bridged depth image and
    # camera_info, is the same data at ~10 MB/s, and nothing subscribes (depth_obstacle_node
    # reprojects the overhead depth image itself, and the perception backend reads the depth
    # image).
    #
    # The three wrist entries stay in the list unconditionally so that `wrist_camera:=false`
    # changes only what is rendered. A bridge for a topic that never publishes is an idle
    # subscription, and a conditional list would need a second parameter_bridge process.
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/overhead_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/wrist_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("cameras")),
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_restocker",
        output="screen",
        parameters=[
            {
                "world": "restocking",
                "string": robot_description,
                "name": "restocker",
                "allow_renaming": False,
            }
        ],
    )
    # The controller manager must not see a time-base discontinuity at its first clocked update;
    # the wait (and that reasoning) lives in restocker_gazebo.wait_for_clock, which gates the
    # controller spawner below on the first /clock message.
    wait_for_simulation_clock = ExecuteProcess(
        cmd=["python3", "-m", "restocker_gazebo.wait_for_clock"],
        name="wait_for_simulation_clock",
        output="log",
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="restocker_controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "arm_controller",
            "rail_controller",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_timeout,
            "--switch-timeout",
            controller_timeout,
            "--service-call-timeout",
            controller_timeout,
            "--activate-as-group",
        ],
    )

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"ERROR: robot spawn failed with exit code {event.returncode}"),
                EmitEvent(event=Shutdown(reason="robot spawn failed")),
            ]
        return [wait_for_simulation_clock]

    def after_clock(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg="ERROR: no /clock message arrived before the controllers were spawned"
                ),
                EmitEvent(event=Shutdown(reason="simulation clock never started")),
            ]
        return [controller_spawner]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Start the Gazebo graphical client"
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("restocker_gazebo"), "worlds", "restocking.sdf"]
                ),
                description="Absolute SDF world path",
            ),
            DeclareLaunchArgument(
                "controller_timeout",
                default_value="30.0",
                description="Controller-manager and activation timeout in seconds",
            ),
            DeclareLaunchArgument(
                "scenario_config",
                default_value=default_scenario,
                description="Validated workcell/product scenario YAML",
            ),
            DeclareLaunchArgument(
                "scenario_seed",
                default_value=FIXED_SCENARIO_SEED,
                description=(
                    "'fixed' to spawn the scenario's surveyed products verbatim, or an unsigned "
                    "64-bit integer to draw a replayable random scenario from its bounds"
                ),
            ),
            DeclareLaunchArgument(
                "product_catalog",
                default_value=default_product_catalog,
                description="Description-owned deterministic product collision catalog",
            ),
            DeclareLaunchArgument(
                "workcell_geometry",
                default_value=default_workcell_geometry,
                description="Description-owned surveyed workcell geometry",
            ),
            DeclareLaunchArgument(
                "attachment_boundary",
                default_value=default_attachment_boundary,
                description="Validated simulator attachment policy YAML",
            ),
            DeclareLaunchArgument(
                "gripper_geometry",
                default_value=default_gripper_geometry,
                description="Description-owned gripper geometry YAML",
            ),
            DeclareLaunchArgument(
                "spawn_scenario",
                default_value="true",
                description="Spawn the shelf, stock tray, and configured products",
            ),
            DeclareLaunchArgument(
                "ground_truth",
                default_value="true",
                description="Publish backend-neutral simulator ground-truth observations",
            ),
            DeclareLaunchArgument(
                "object_observation_topic",
                default_value="/perception/object_observations",
                description=(
                    "Where simulator ground truth publishes object poses. It defaults to the "
                    "topic the world state ingests; move it to "
                    "/perception/ground_truth/object_observations to demote ground truth to "
                    "evaluation evidence and let perception produce instead"
                ),
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="true",
                description="Bridge the RGB-D camera image, depth and camera_info topics",
            ),
            DeclareLaunchArgument(
                "wrist_camera",
                default_value="true",
                description=(
                    "Render the wrist RGB-D sensor. On by default; set false to buy back renderer "
                    "time in runs that never look through this camera"
                ),
            ),
            DeclareLaunchArgument(
                "bundle_scenario",
                default_value="false",
                description=(
                    "Place the workcell and every product in the world file Gazebo loads, so "
                    "they stand there from the first frame, instead of spawning them one create "
                    "process at a time over the first half minute"
                ),
            ),
            DeclareLaunchArgument(
                "scene_materials",
                default_value="",
                description=(
                    "With bundle_scenario: a YAML file restyling the visuals of bundled models "
                    "(see restocker_gazebo.world_bundle), so a dressed scene need not change the "
                    "description every test shares"
                ),
            ),
            # Must precede every action that reads `scenario_config`: a seeded run rewrites it to
            # the generated document.
            OpaqueFunction(function=_resolve_scenario),
            OpaqueFunction(function=_bundle_world),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                [
                    PathJoinSubstitution([FindPackagePrefix("restocker_gazebo"), "lib"]),
                    ":",
                    EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "RESTOCKER_ATTACHMENT_BOUNDARY_CONFIG",
                LaunchConfiguration("attachment_boundary"),
            ),
            SetEnvironmentVariable(
                "RESTOCKER_GRIPPER_GEOMETRY_CONFIG",
                LaunchConfiguration("gripper_geometry"),
            ),
            SetEnvironmentVariable(
                "RESTOCKER_PRODUCT_CATALOG_CONFIG",
                LaunchConfiguration("product_catalog"),
            ),
            SetEnvironmentVariable(
                "RESTOCKER_SCENARIO_CONFIG", LaunchConfiguration("scenario_config")
            ),
            gazebo_gui,
            gazebo_headless,
            state_publisher,
            world_to_rail,
            clock_bridge,
            camera_bridge,
            spawn_robot,
            OpaqueFunction(function=_scenario_actions),
            RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=after_spawn)),
            RegisterEventHandler(
                OnProcessExit(target_action=wait_for_simulation_clock, on_exit=after_clock)
            ),
        ]
    )
