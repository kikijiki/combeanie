# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Public composition root for the Gazebo and ros2_control simulation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    OrSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Forward the stable public arguments to the simulator-owned launch."""
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    controller_timeout = LaunchConfiguration("controller_timeout")
    scenario_config = LaunchConfiguration("scenario_config")
    scenario_seed = LaunchConfiguration("scenario_seed")
    product_catalog = LaunchConfiguration("product_catalog")
    workcell_geometry = LaunchConfiguration("workcell_geometry")
    lane_semantics = LaunchConfiguration("lane_semantics")
    lane_policy_state = LaunchConfiguration("lane_policy_state")
    placement_require_column_growth = LaunchConfiguration("placement_require_column_growth")
    spawn_scenario = LaunchConfiguration("spawn_scenario")
    ground_truth = LaunchConfiguration("ground_truth")
    ground_truth_object_topic = LaunchConfiguration("object_observation_topic")
    lane_observation_topic = LaunchConfiguration("lane_observation_topic")
    lane_evidence_validity_ms = LaunchConfiguration("lane_evidence_validity_ms")
    maximum_observation_age_ms = LaunchConfiguration("maximum_observation_age_ms")
    cameras = LaunchConfiguration("cameras")
    # Without the camera nothing produces /perception/object_observations (ground truth publishes
    # elsewhere), so a composition with cameras off runs no perception.
    perception = AndSubstitution(LaunchConfiguration("cameras"), LaunchConfiguration("perception"))
    perception_config = PathJoinSubstitution(
        [FindPackageShare("restocker_perception"), "config", "overhead_rgbd_perception.yaml"]
    )
    # The lane producer and its evaluator both need the wrist renderer, so the gate is the
    # conjunction.
    lane_survey = AndSubstitution(
        LaunchConfiguration("wrist_camera"), LaunchConfiguration("lane_survey")
    )
    lane_survey_config = PathJoinSubstitution(
        [FindPackageShare("restocker_perception"), "config", "lane_survey.yaml"]
    )
    # Wrist tray overview / confirm reuse perception_node with their own parameter file. Off by
    # default and gated on the wrist renderer. The overview duty publishes candidates on
    # /perception/tray_candidates, which the world state never ingests, so it cannot collide
    # with ground truth; only the confirm duty publishes on the admitted object topic, so
    # turning it on also needs object_observation_topic moved.
    tray_overview_perception = AndSubstitution(
        LaunchConfiguration("wrist_camera"), LaunchConfiguration("tray_overview_perception")
    )
    tray_confirm_perception = AndSubstitution(
        LaunchConfiguration("wrist_camera"), LaunchConfiguration("tray_confirm_perception")
    )
    tray_perception_active = OrSubstitution(tray_overview_perception, tray_confirm_perception)
    tray_perception_config = PathJoinSubstitution(
        [FindPackageShare("restocker_perception"), "config", "wrist_tray_perception.yaml"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "world": world,
            "controller_timeout": controller_timeout,
            "scenario_config": scenario_config,
            "scenario_seed": scenario_seed,
            "product_catalog": product_catalog,
            "workcell_geometry": workcell_geometry,
            "spawn_scenario": spawn_scenario,
            "ground_truth": ground_truth,
            "cameras": cameras,
            "wrist_camera": LaunchConfiguration("wrist_camera"),
            "object_observation_topic": ground_truth_object_topic,
        }.items(),
    )
    world_state = Node(
        package="restocker_world_state",
        executable="world_state_node",
        name="world_state",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "lane_semantics_config": lane_semantics,
                # Durable desired-stocking document; repoint it alongside lane_semantics so a
                # saved intent for one policy never overlays another.
                "lane_policy_state": lane_policy_state,
                "workcell_geometry": workcell_geometry,
                # Lane depth that one more of each catalogued product costs.
                "product_catalog": product_catalog,
                # Camera confidence is the posterior that a blob is its classified product: 0.71 to
                # 0.86 on this cell's three products (floor set by the translucent small bottle).
                # 0.60 is below every valid observation and above 0.50, the score of a blob on the
                # colour acceptance boundary. Simulator ground truth publishes 1.0 and would pass
                # the node's 0.99 default. Not the only gate: the geometric fit in
                # restocker_perception yields no observation at all for a bad fit.
                "minimum_confidence": 0.60,
                # Simulator ground truth by default. The camera-derived producer publishes on
                # /perception/lane_observations so the two can be compared without changing the
                # executor's evidence.
                "lane_observation_topic": lane_observation_topic,
                "lane_evidence_validity_ms": ParameterValue(
                    lane_evidence_validity_ms, value_type=int
                ),
                "maximum_observation_age_ms": ParameterValue(
                    maximum_observation_age_ms, value_type=int
                ),
                "placement_require_column_growth": ParameterValue(
                    placement_require_column_growth, value_type=bool
                ),
            }
        ],
    )
    # UDP-only for this subscriber, paired with the same opt-in on the clock-bridge publisher in
    # restocker_gazebo's simulation.launch: Fast DDS shared-memory port files are not
    # domain-scoped, and the measured cross-domain init failures landed on both ends of the
    # /clock hop (bridge publisher and this world-state subscriber). Camera bridges and image
    # topics keep shared memory — lease-wide UDP was tried and the first wave running it
    # produced jam-class controller aborts no shared-memory wave had shown (see
    # scripts/domain_isolation.py).
    world_state = GroupAction(
        [
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            world_state,
        ]
    )
    # Camera-derived lane producer: one acquisition per request, on the topic the world state can
    # ingest via lane_observation_topic. It does not move the arm; restocker_task_executor's
    # lane_survey_node aims the camera and calls this node's service.
    lane_observation_node = Node(
        package="restocker_perception",
        executable="lane_observation_node",
        name="lane_observation",
        output="screen",
        parameters=[
            lane_survey_config,
            {"use_sim_time": True, "workcell_geometry": workcell_geometry},
        ],
        condition=IfCondition(lane_survey),
    )
    # Evaluation only: the sole subscriber to both lane topics, publishes nothing the execution
    # path reads.
    lane_depth_error_evaluator = Node(
        package="restocker_perception",
        executable="lane_depth_error_evaluator_node",
        name="lane_depth_error_evaluator",
        output="screen",
        parameters=[lane_survey_config, {"use_sim_time": True}],
        condition=IfCondition(lane_survey),
    )
    # The producer of /perception/object_observations (ground truth publishes elsewhere): what the
    # world state ingests and the executor plans against.
    perception_node = Node(
        package="restocker_perception",
        executable="perception_node",
        name="perception",
        output="screen",
        parameters=[
            perception_config,
            {
                "use_sim_time": True,
                # Names come from the scenario inventory: the simulated attachment adapter resolves
                # a reserved source identity through an immutable scenario mapping, and a camera
                # cannot see a model name.
                "object_manifest_path": scenario_config,
            },
        ],
        condition=IfCondition(perception),
    )
    # Tray overview duty: same node binary, wrist topics, tray-elevation parameters.
    tray_overview_perception_node = Node(
        package="restocker_perception",
        executable="perception_node",
        name="tray_overview_perception",
        output="screen",
        parameters=[
            tray_perception_config,
            {
                "use_sim_time": True,
                "object_manifest_path": scenario_config,
            },
        ],
        condition=IfCondition(tray_overview_perception),
    )
    # Tray confirm duty: closer depth band and a higher pixel floor, same elevation.
    tray_confirm_perception_node = Node(
        package="restocker_perception",
        executable="perception_node",
        name="tray_confirm_perception",
        output="screen",
        parameters=[
            tray_perception_config,
            {
                "use_sim_time": True,
                "object_manifest_path": scenario_config,
            },
        ],
        condition=IfCondition(tray_confirm_perception),
    )
    # Evaluation only: the sole subscriber to the ground-truth topic. Do not enable overhead
    # perception and a tray duty together: both publish object observations and would start two
    # evaluators on the same sample topic.
    pose_error_evaluator = Node(
        package="restocker_perception",
        executable="pose_error_evaluator_node",
        name="pose_error_evaluator",
        output="screen",
        parameters=[perception_config, {"use_sim_time": True}],
        condition=IfCondition(perception),
    )
    tray_pose_error_evaluator = Node(
        package="restocker_perception",
        executable="pose_error_evaluator_node",
        name="tray_pose_error_evaluator",
        output="screen",
        parameters=[tray_perception_config, {"use_sim_time": True}],
        condition=IfCondition(tray_perception_active),
    )
    telemetry_adapter = Node(
        package="restocker_control",
        executable="joint_state_telemetry_adapter",
        name="joint_state_telemetry_adapter",
        output="screen",
        parameters=[{"use_sim_time": True}],
        on_exit=Shutdown(reason="controller telemetry adapter exited"),
    )

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
                default_value=PathJoinSubstitution(
                    [FindPackageShare("restocker_gazebo"), "config", "baseline_products.yaml"]
                ),
                description="Validated workcell/product scenario YAML",
            ),
            DeclareLaunchArgument(
                "product_catalog",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "product_collision_catalog.yaml",
                    ]
                ),
                description="Description-owned deterministic product collision catalog",
            ),
            DeclareLaunchArgument(
                "workcell_geometry",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "workcell_geometry.yaml",
                    ]
                ),
                description="Description-owned surveyed workcell geometry",
            ),
            DeclareLaunchArgument(
                "lane_semantics",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("restocker_world_state"), "config", "baseline_lanes.yaml"]
                ),
                description="World-state-owned lane compatibility policy",
            ),
            DeclareLaunchArgument(
                "lane_policy_state",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_world_state"),
                        "config",
                        "lane_policy_state.yaml",
                    ]
                ),
                description=(
                    "Durable SetLanePolicy document; overlays lane_semantics at start-up. "
                    "Keep it paired with the lane_semantics it was written for"
                ),
            ),
            DeclareLaunchArgument(
                "placement_require_column_growth",
                default_value="true",
                description=(
                    "Placement proof selector: true proves a placement by post-release "
                    "column growth of one pitch less tolerance (Milestone 10 section 3, "
                    "works from camera lane geometry alone); false proves it by "
                    "post-release target identity instead (ground-truth producers)"
                ),
            ),
            DeclareLaunchArgument(
                "scenario_seed",
                default_value="fixed",
                description=(
                    "Scenario selection: 'fixed' spawns the configured products verbatim, "
                    "or an integer seed generates a repeatable randomized scenario"
                ),
            ),
            DeclareLaunchArgument("spawn_scenario", default_value="true"),
            DeclareLaunchArgument("ground_truth", default_value="true"),
            DeclareLaunchArgument("cameras", default_value="true"),
            DeclareLaunchArgument("wrist_camera", default_value="true"),
            DeclareLaunchArgument(
                "object_observation_topic",
                default_value="/perception/object_observations",
                description=(
                    "Where simulator ground truth publishes object poses. It defaults to the "
                    "topic the world state ingests; move it to "
                    "/perception/ground_truth/object_observations, with perception:=true, to put "
                    "the camera in front of the executor instead"
                ),
            ),
            DeclareLaunchArgument(
                "lane_evidence_validity_ms",
                default_value="60000",
                description=(
                    "World-state validity horizon on a lane's depletion evidence, "
                    "milliseconds; baseline.launch.py feeds the same value to selection and "
                    "to the campaign's transfer-boundary re-check"
                ),
            ),
            DeclareLaunchArgument(
                "maximum_observation_age_ms",
                default_value="2000",
                description=(
                    "World-state age bound on the selected product's observation at "
                    "reservation, milliseconds; baseline.launch.py documents why "
                    "sensor-driven runs raise it"
                ),
            ),
            DeclareLaunchArgument(
                "perception",
                default_value="false",
                description=(
                    "Run the RGB-D perception pipeline that produces object observations, and the "
                    "evaluator that measures the translation error against simulator ground "
                    "truth. Off by default: what remains is motion execution under load, not a "
                    "perception boundary, and the backend estimates no orientation. "
                    "Turning it on also needs object_observation_topic "
                    "moved, or both would publish to the same topic"
                ),
            ),
            DeclareLaunchArgument(
                "lane_observation_topic",
                default_value="/perception/ground_truth/lane_observations",
                description=(
                    "Which lane-observation topic the world state ingests. Defaults to simulator "
                    "ground truth (continuous). The wrist camera survey publishes on "
                    "/perception/lane_observations for the lane-depth error evaluator; point "
                    "this there only when that producer is feeding the lanes selection needs"
                ),
            ),
            DeclareLaunchArgument(
                "lane_survey",
                default_value="true",
                description=(
                    "Run the wrist-camera lane depletion producer and the evaluator that measures "
                    "it against simulator ground truth. Needs wrist_camera. Publishes on "
                    "/perception/lane_observations; world state still defaults to ground truth "
                    "until that producer is selected via lane_observation_topic"
                ),
            ),
            DeclareLaunchArgument(
                "tray_overview_perception",
                default_value="false",
                description=(
                    "Run ColourDepthBackend on the wrist camera with tray-overview parameters "
                    "(51-degree elevation, ~0.4-1.0 m range), publishing candidates on "
                    "/perception/tray_candidates — never on the world state's ingest topic. "
                    "Off by default. Needs wrist_camera"
                ),
            ),
            DeclareLaunchArgument(
                "tray_confirm_perception",
                default_value="false",
                description=(
                    "Run ColourDepthBackend on the wrist camera with tray-confirm parameters "
                    "(same elevation, 0.25-0.35 m standoff band) on "
                    "/perception/object_observations. Off by default. Needs wrist_camera and "
                    "object_observation_topic moved to ground truth, or the world state's "
                    "single-publisher pin refuses both"
                ),
            ),
            simulation,
            telemetry_adapter,
            world_state,
            perception_node,
            tray_overview_perception_node,
            tray_confirm_perception_node,
            pose_error_evaluator,
            tray_pose_error_evaluator,
            lane_observation_node,
            lane_depth_error_evaluator,
        ]
    )
