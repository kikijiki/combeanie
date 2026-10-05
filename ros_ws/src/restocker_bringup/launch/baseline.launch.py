# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Public composition root for simulation, control, and MoveIt planning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    OrSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from restocker_moveit_config import build_moveit_config, parallel_planning_parameters


def generate_launch_description() -> LaunchDescription:
    """Compose existing subsystem launches and forward their stable arguments."""
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    # Demo scope: defaults to the MoveIt layout, so every launch and test that does not pass
    # this argument renders exactly the windows it did before. scripts/demo.bash points it at
    # restocker_bringup/rviz/demo_camera_view.rviz to add the camera panes.
    rviz_config = LaunchConfiguration("rviz_config")
    world = LaunchConfiguration("world")
    controller_timeout = LaunchConfiguration("controller_timeout")
    planning_smoke = LaunchConfiguration("planning_smoke")
    planning_timeout = LaunchConfiguration("planning_timeout")
    scenario_config = LaunchConfiguration("scenario_config")
    scenario_seed = LaunchConfiguration("scenario_seed")
    product_catalog = LaunchConfiguration("product_catalog")
    workcell_geometry = LaunchConfiguration("workcell_geometry")
    lane_semantics = LaunchConfiguration("lane_semantics")
    lane_policy_state = LaunchConfiguration("lane_policy_state")
    placement_require_column_growth = LaunchConfiguration("placement_require_column_growth")
    spawn_scenario = LaunchConfiguration("spawn_scenario")
    ground_truth = LaunchConfiguration("ground_truth")
    perception = LaunchConfiguration("perception")
    ground_truth_object_topic = LaunchConfiguration("object_observation_topic")
    planning_scene_projection = LaunchConfiguration("planning_scene_projection")
    projector_log_level = LaunchConfiguration("projector_log_level")
    product_observation_max_age = LaunchConfiguration("product_observation_max_age")
    selection_object_max_age_ms = LaunchConfiguration("selection_object_max_age_ms")
    lane_evidence_validity_ms = LaunchConfiguration("lane_evidence_validity_ms")
    maximum_observation_age_ms = LaunchConfiguration("maximum_observation_age_ms")
    task_execution_timeout_ms = LaunchConfiguration("task_execution_timeout_ms")
    cameras = LaunchConfiguration("cameras")
    obstacle_perception = LaunchConfiguration("obstacle_perception")
    # The depth pipeline needs the overhead camera, and the projector must not *require* evidence
    # no producer can deliver: cameras:=false with require_obstacle_evidence left on would leave
    # the scene permanently uncertified (a fail-closed deadlock). Same conjunction pattern
    # simulation.launch.py uses for `perception`.
    obstacle_evidence_required = AndSubstitution(cameras, obstacle_perception)
    obstacle_depth_reliable = LaunchConfiguration("obstacle_depth_reliable")
    attachment_adapter = LaunchConfiguration("attachment_adapter")
    task_coordinator = LaunchConfiguration("task_coordinator")
    survey_viewpoint = LaunchConfiguration("survey_viewpoint")
    lane_survey = LaunchConfiguration("lane_survey")
    tray_survey = LaunchConfiguration("tray_survey")
    autonomous_campaign = LaunchConfiguration("autonomous_campaign")
    campaign_max_cycles = LaunchConfiguration("campaign_max_cycles")
    campaign_cycle_period = LaunchConfiguration("campaign_cycle_period")
    campaign_restart_acknowledged = LaunchConfiguration("campaign_restart_acknowledged")
    # The tray survey is a client of the viewpoint survey action, so enabling it enables the
    # server it composes: one motion coordinator, never two.
    survey_viewpoint_active = OrSubstitution(
        OrSubstitution(survey_viewpoint, autonomous_campaign), tray_survey
    )
    lane_survey_active = OrSubstitution(lane_survey, autonomous_campaign)
    # The mode loop's SURVEY_TRAY path is the coordinated SurveyTray action, so running the
    # campaign also starts its server beside the survey servers it already composes.
    tray_survey_active = OrSubstitution(tray_survey, autonomous_campaign)
    motion_enabled = LaunchConfiguration("motion_enabled")
    reasoner_enabled = LaunchConfiguration("reasoner_enabled")
    reasoner_model = LaunchConfiguration("reasoner_model")
    reasoner_audit_path = LaunchConfiguration("reasoner_audit_path")
    moveit_config = build_moveit_config()
    # Card 053: the free-space race (ompl RRTConnect + ompl_fallback PRM) runs inside the
    # motion ports, so the three port-hosting nodes receive both pipeline namespaces.
    free_space_race_parameters = parallel_planning_parameters()

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "simulation.launch.py"]
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
            "lane_semantics": lane_semantics,
            "lane_policy_state": lane_policy_state,
            "placement_require_column_growth": placement_require_column_growth,
            "spawn_scenario": spawn_scenario,
            "ground_truth": ground_truth,
            "perception": perception,
            # Forwarded so compositions above the simulator can disable the renderers, and so the
            # obstacle-evidence gate below agrees with what the simulation actually runs.
            "cameras": cameras,
            # Forwarded so compositions above the simulator can disable the wrist renderer.
            "wrist_camera": LaunchConfiguration("wrist_camera"),
            "object_observation_topic": ground_truth_object_topic,
            "lane_observation_topic": LaunchConfiguration("lane_observation_topic"),
            "lane_evidence_validity_ms": lane_evidence_validity_ms,
            "maximum_observation_age_ms": maximum_observation_age_ms,
            "lane_survey": lane_survey_active,
            "tray_overview_perception": LaunchConfiguration("tray_overview_perception"),
            "tray_confirm_perception": LaunchConfiguration("tray_confirm_perception"),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_moveit_config"), "launch", "move_group.launch.py"]
            )
        ),
        launch_arguments={
            "rviz": rviz,
            "rviz_config": rviz_config,
            "use_sim_time": "true",
        }.items(),
    )
    smoke_client = Node(
        package="restocker_task_executor",
        executable="planning_smoke_test",
        name="planning_smoke_test",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "planning_time_sec": ParameterValue(planning_timeout, value_type=float),
            },
        ],
        condition=IfCondition(planning_smoke),
        on_exit=Shutdown(reason="planning smoke client finished"),
    )
    workcell_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "urdf",
                        "workcell.urdf.xacro",
                    ]
                ),
                " geometry_config:=",
                workcell_geometry,
            ]
        ),
        value_type=str,
    )
    scene_projector = Node(
        package="restocker_task_executor",
        executable="planning_scene_projector_node",
        name="planning_scene_projector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "workcell_description": workcell_description,
                "product_catalog_path": product_catalog,
                "max_observation_age_sec": ParameterValue(
                    product_observation_max_age, value_type=float
                ),
                # A product inside a lane is projected as part of the lane's occupied volume (the
                # surveyed cross-section over the depth its evidence reports as full), not as an
                # object.
                "workcell_geometry_path": workcell_geometry,
                # Card 050: seed every product the scenario document declares into the scene until
                # world state tracks it (Milestone 10 §6), so a transfer cannot be planned through
                # an unconfirmed tray product. Coupled to spawn_scenario: a launch that does not
                # spawn this document must clear this parameter too.
                "scenario_path": scenario_config,
                # With the depth pipeline running, missing obstacle evidence is a degradation from
                # the first cycle. The projector's default is the weaker latch, for deployments
                # with no depth sensor. Gated on cameras as well: with the renderers off no
                # producer can ever satisfy the requirement, so requiring it would certify
                # nothing forever.
                "require_obstacle_evidence": ParameterValue(
                    obstacle_evidence_required, value_type=bool
                ),
            }
        ],
        # At debug level the projector logs each service stage's round trip.
        arguments=[
            "--ros-args",
            "--log-level",
            ["planning_scene_projector:=", projector_log_level],
        ],
        condition=IfCondition(planning_scene_projection),
    )
    # Volume of interest: the traverse corridor between the stock tray and the shelf face, in the
    # world frame. Derived from restocker_description/config/workcell_geometry.yaml with the
    # workcell at y = 0.55 (restocker_gazebo/config/baseline_products.yaml): the shelf front face
    # is at y = 0.10 and the stock tray reaches y = -0.525. The floor is excluded at z = 0.20,
    # above the rail, because MoveIt already models the rail.
    #
    # With the cell empty the only returns inside the volume are the robot itself (|x| <= 0.25);
    # the nearest unmodelled structures, the tray rim (y = -0.52, z = 0.57) and the shelf front
    # face (returns at y > 0.08), fall outside.
    #
    # The self-filter is not configured here: depth_obstacle_node models each link as a capsule
    # derived from that link's collision solid in restocker_description/urdf (see the capsule
    # table in depth_obstacle_node.cpp), and its defaults are that derivation.
    #
    # Measured with the capsules (160 observations at each of four poses: arm parked, arm folded
    # across the corridor, rail at -1.69 and +1.69): 0 false-positive robot_static boxes.
    # A 0.15 x 0.15 x 0.50 box swept toward the parked arm down to 0.15 m standoff was reported as
    # one box with every face within 0.03 m of truth (the base being the least accurate face).
    # 0.30 m is the rail clearance test_dynamic_obstacle_runtime.py relies on.
    #
    # The extractor's vertical bridging keeps one obstacle as one box; without it one obstacle
    # came back as 5 stacked boxes, up to max_boxes.
    #
    # The condition matches require_obstacle_evidence above: no cameras, no detector, no
    # requirement — never a required-evidence projector with nothing that can feed it.
    depth_obstacle_node = Node(
        package="restocker_perception",
        executable="depth_obstacle_node",
        name="depth_obstacle_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "planning_frame": "world",
                "volume_of_interest_min_xyz_m": [-1.60, -0.52, 0.20],
                "volume_of_interest_max_xyz_m": [1.60, 0.08, 1.60],
                "reliable_sensor_qos": ParameterValue(obstacle_depth_reliable, value_type=bool),
            }
        ],
        condition=IfCondition(obstacle_evidence_required),
    )
    attachment_adapter_node = Node(
        package="restocker_gazebo",
        executable="attachment_adapter_node",
        name="simulation_attachment_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "scenario_path": scenario_config,
                "product_catalog_path": product_catalog,
            }
        ],
        condition=IfCondition(attachment_adapter),
    )
    coordinator_config = PathJoinSubstitution(
        [
            FindPackageShare("restocker_task_executor"),
            "config",
            "restock_action_coordinator.yaml",
        ]
    )
    gripper_geometry = PathJoinSubstitution(
        [FindPackageShare("restocker_description"), "config", "gripper_geometry.yaml"]
    )
    task_coordinator_node = Node(
        package="restocker_task_executor",
        executable="restock_action_coordinator",
        name="restock_action_coordinator",
        output="screen",
        parameters=[
            coordinator_config,
            # The motion port builds its own MoveGroupInterface, which reads the robot model from
            # this node's parameters.
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            free_space_race_parameters,
            {
                "use_sim_time": True,
                "product_catalog_path": product_catalog,
                "workcell_geometry_path": workcell_geometry,
                "gripper_geometry_path": gripper_geometry,
                "motion_enabled": ParameterValue(motion_enabled, value_type=bool),
                "selection.maximum_object_age_ms": ParameterValue(
                    selection_object_max_age_ms, value_type=int
                ),
                "selection.lane_evidence_validity_ms": ParameterValue(
                    lane_evidence_validity_ms, value_type=int
                ),
                "task.execution_timeout_ms": ParameterValue(
                    task_execution_timeout_ms, value_type=int
                ),
                # With no projector there is no scene authority to consult and the motion port
                # would refuse every segment, so the gate follows the projector.
                "require_planning_scene_authority": ParameterValue(
                    planning_scene_projection, value_type=bool
                ),
                # Off unless asked for. The advisory reasoner can only decline a recovery the
                # deterministic authorisation already permitted.
                "reasoner.enabled": ParameterValue(reasoner_enabled, value_type=bool),
                "reasoner.model": ParameterValue(reasoner_model, value_type=str),
                "reasoner.audit_path": ParameterValue(reasoner_audit_path, value_type=str),
            },
        ],
        condition=IfCondition(task_coordinator),
        on_exit=Shutdown(reason="restock action coordinator exited"),
    )

    # Aims the wrist camera. Read-only: plans and executes free-space segments through the same
    # motion port and authority gate as the coordinator and mutates no world state. Off unless
    # asked for. Its motion port and the coordinator's are two clients of one move_group, so
    # compositions that use it leave the coordinator off.
    survey_viewpoint_node = Node(
        package="restocker_task_executor",
        executable="survey_viewpoint_node",
        name="survey_viewpoint",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            free_space_race_parameters,
            {
                "use_sim_time": True,
                "workcell_geometry_path": workcell_geometry,
                # As for the coordinator.
                "require_planning_scene_authority": ParameterValue(
                    planning_scene_projection, value_type=bool
                ),
            },
        ],
        condition=IfCondition(survey_viewpoint_active),
        on_exit=Shutdown(reason="survey viewpoint node exited"),
    )

    # Looks down one lane: aims through the viewpoint survey, then asks restocker_perception's
    # lane_observation node for one acquisition. Read-only under the authority gate; like the
    # viewpoint survey, it excludes the coordinator.
    lane_survey_node = Node(
        package="restocker_task_executor",
        executable="lane_survey_node",
        name="lane_survey",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            free_space_race_parameters,
            {
                "use_sim_time": True,
                "workcell_geometry_path": workcell_geometry,
                "require_planning_scene_authority": ParameterValue(
                    planning_scene_projection, value_type=bool
                ),
            },
        ],
        condition=IfCondition(lane_survey_active),
        on_exit=Shutdown(reason="lane survey node exited"),
    )

    # Coordinates the two-tier tray survey: overview stations through /survey_viewpoint, then
    # one confirmation viewpoint for one selected candidate. Read-only — it publishes no
    # observation and mutates no world state; overview candidates stay on
    # /perception/tray_candidates and only the confirm duty writes the admitted object topic.
    # Excludes the coordinator, as every survey server does.
    tray_survey_node = Node(
        package="restocker_task_executor",
        executable="tray_survey_node",
        name="tray_survey",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "workcell_geometry_path": workcell_geometry,
            },
        ],
        condition=IfCondition(tray_survey_active),
        on_exit=Shutdown(reason="tray survey node exited"),
    )

    # Continuous behaviour above the three authority boundaries. It serialises all arm users: a
    # front sweep, the less frequent back sweep, then as many RestockProduct goals as the
    # evidence supports — a confirming cycle names its confirmed product, a skip-path cycle
    # leaves the pair to selection. Enabling it also enables both survey servers; benchmarks
    # leave it off to keep direct goal-count control.
    autonomous_campaign_node = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name="autonomous_restock_campaign",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "workcell_geometry_path": workcell_geometry,
                "product_catalog_path": product_catalog,
                "max_cycles": ParameterValue(campaign_max_cycles, value_type=int),
                "cycle_period_sec": ParameterValue(campaign_cycle_period, value_type=float),
                # Card 086 stage 1: unresolved-motion state is in memory only, so the campaign
                # waits for this acknowledgment before its first goal after any (re)start.
                "restart_acknowledged": ParameterValue(
                    campaign_restart_acknowledged, value_type=bool
                ),
                "lane_evidence_validity_ms": ParameterValue(
                    lane_evidence_validity_ms, value_type=int
                ),
                # The coordinator's selection horizon, from the same argument as its
                # selection.maximum_object_age_ms: the campaign's retired-ghost gate must cover
                # every object an unpinned transfer may still pick (Milestone 10 §6, Card 069).
                "coordinator_object_max_age_ms": ParameterValue(
                    selection_object_max_age_ms, value_type=int
                ),
            }
        ],
        condition=IfCondition(autonomous_campaign),
        on_exit=Shutdown(reason="autonomous campaign node exited"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui", default_value="true", description="Start the Gazebo graphical client"
            ),
            DeclareLaunchArgument("rviz", default_value="true", description="Start MoveIt RViz"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("restocker_moveit_config"), "rviz", "moveit.rviz"]
                ),
                description=(
                    "Absolute RViz configuration path. The default is the MoveIt planning "
                    "view; scripts/demo.bash selects restocker_bringup's "
                    "rviz/demo_camera_view.rviz, which adds docked wrist and overhead camera "
                    "panes on the bridged colour topics"
                ),
            ),
            DeclareLaunchArgument(
                "motion_enabled",
                default_value="true",
                description="Let the restock coordinator plan and execute pre-grasp motion",
            ),
            DeclareLaunchArgument(
                "reasoner_enabled",
                default_value="false",
                description="Compose the optional advisory recovery reasoner",
            ),
            DeclareLaunchArgument(
                "reasoner_model",
                default_value="",
                description="Model name to ask the advisory backend for",
            ),
            DeclareLaunchArgument(
                "reasoner_audit_path",
                default_value="",
                description="JSON Lines file for advisory decision records; empty writes none",
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
                "product_observation_max_age",
                default_value="600.0",
                description=(
                    "Product evidence age beyond which scene projection reports a product "
                    "as aged, seconds. An aged product stays in the scene at its last known "
                    "pose; age never fails the projection (Card 071). Shipped for the "
                    "sensor-driven default (Milestone 10 Stage 7): a camera product is "
                    "observed on a survey, not continuously. Simulator ground truth refreshes "
                    "continuously and is unaffected"
                ),
            ),
            DeclareLaunchArgument(
                "selection_object_max_age_ms",
                default_value="180000",
                description=(
                    "Maximum product evidence age accepted for task selection, milliseconds. "
                    "Shipped for the sensor-driven default: the measured cold-start span "
                    "(sweep ~35 s + tray overview/confirm ~45 s) must not expire a candidate "
                    "between the survey that found it and the transfer that uses it"
                ),
            ),
            DeclareLaunchArgument(
                "lane_evidence_validity_ms",
                default_value="180000",
                description=(
                    "How long a surveyed lane's depletion evidence may be acted on without a "
                    "re-survey, milliseconds. Feeds task selection, the world state's "
                    "reservation gate, and the campaign's transfer-boundary re-check alike. "
                    "Shipped for the sensor-driven default: the measured cold-start span of "
                    "one shelf sweep plus one coordinated tray confirm is about 105 s, so a "
                    "sensor-driven run that surveys before its first confirm needs this above "
                    "that span; simulator ground truth refreshes lanes continuously and does "
                    "not care"
                ),
            ),
            DeclareLaunchArgument(
                "maximum_observation_age_ms",
                default_value="180000",
                description=(
                    "Maximum age of the selected product's observation accepted at "
                    "reservation, milliseconds. Shipped for the sensor-driven default: the "
                    "confirm stamp predates the coordinated survey's result by its acquisition "
                    "window (about 4 s), and a transfer-boundary re-survey can age it by one "
                    "sweep (about 47 s); the default sits above both where it used to be the "
                    "shipped 2 s reservation gate"
                ),
            ),
            DeclareLaunchArgument(
                "task_execution_timeout_ms",
                default_value="180000",
                description="Absolute execution budget for one planned motion segment",
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
            DeclareLaunchArgument(
                "cameras",
                default_value="true",
                description=(
                    "Render the overhead and wrist camera sensors and bridge them. Obstacle "
                    "perception composes only while this is on: with the cameras off there is "
                    "no depth producer, so the evidence requirement is dropped with them "
                    "instead of certifying nothing forever"
                ),
            ),
            DeclareLaunchArgument(
                "wrist_camera",
                default_value="true",
                description=(
                    "Render the wrist RGB-D sensor. Set false to save renderer time in runs that "
                    "never look through this camera"
                ),
            ),
            DeclareLaunchArgument(
                "perception",
                default_value="false",
                description=(
                    "Produce object observations from the overhead RGB-D camera. Off by "
                    "default: the wrist tray duties are the shipped camera source for the "
                    "sensor-driven default. It publishes on the world state's ingest topic, so "
                    "turning it on needs tray_confirm_perception off (or its topic moved), or "
                    "the single-publisher pin refuses both producers"
                ),
            ),
            DeclareLaunchArgument(
                "object_observation_topic",
                default_value="/perception/ground_truth/object_observations",
                description=(
                    "Where simulator ground truth publishes object poses. Shipped demoted "
                    "away from the topic the world state ingests "
                    "(/perception/object_observations), which the wrist tray duties own: with "
                    "tray_confirm_perception on by default the single-publisher pin admits one "
                    "producer. Point it back at /perception/object_observations with the tray "
                    "duties off to run the ground-truth path"
                ),
            ),
            DeclareLaunchArgument(
                "planning_scene_projection",
                default_value="true",
                description="Project verified world state and workcell geometry into MoveIt",
            ),
            DeclareLaunchArgument(
                "projector_log_level",
                default_value=EnvironmentVariable(
                    "RESTOCKER_PROJECTOR_LOG_LEVEL", default_value="info"
                ),
                description=(
                    "Log level for the planning-scene projector; debug adds a measured "
                    "round-trip record for every service stage"
                ),
            ),
            # On by default (Card 023). Turning it on also sets require_obstacle_evidence, so the
            # scene stops being certified once the newest observation is older than
            # obstacle_max_age_sec (2.0 s). Both the detector and the requirement additionally
            # gate on `cameras` (obstacle_evidence_required above), so cameras:=false disables
            # the pipeline instead of demanding evidence no producer can send.
            #
            # A stale-evidence degradation is treated by the authority gate as a condition that
            # settles: the motion port polls it to a bounded budget (longer than the ~1.2 s
            # degraded window the reliable stream's tail can produce, still expiring on a dead
            # stream). Nothing executes against an uncertified scene. See
            # pregrasp_planning_authority.cpp.
            #
            # Until the first observation arrives the scene is uncertified for that startup gap
            # (measured on quiet runs; attachment startup waits it out rather than failing).
            #
            # test_dynamic_obstacle_runtime.py pins this to false: it publishes its own
            # observations on /perception/obstacle_observations and must be that topic's only
            # publisher, or depth_obstacle_node would keep the evidence it stops from going
            # stale. test_obstacle_perception_runtime.py exercises the composed path with it on.
            DeclareLaunchArgument(
                "obstacle_perception",
                default_value="true",
                description=(
                    "Recover unmodeled obstacles from the overhead depth stream and require "
                    "that evidence before the planning scene is certified; composes only "
                    "while cameras is true"
                ),
            ),
            # The overhead depth image is 2.77 MB and best-effort transport loses large messages in
            # proportion to size. Selects reliable or best-effort subscription; a bridge publishing
            # best-effort is invisible to a reliable subscription, so this stays switchable.
            #
            # Defaults true: the fail-closed projection is judged on the age of the newest
            # observation, and best-effort gaps reached 2.8 s against the 2.0 s window while
            # reliable's widest gap was one 0.167 s frame period.
            DeclareLaunchArgument(
                "obstacle_depth_reliable",
                default_value="true",
                description=(
                    "Subscribe the obstacle detector to the overhead depth stream reliably "
                    "instead of best-effort"
                ),
            ),
            DeclareLaunchArgument(
                "attachment_adapter",
                default_value="true",
                description="Authorize and reconcile simulated physical attachments",
            ),
            DeclareLaunchArgument(
                "task_coordinator",
                default_value="true",
                description="Start the fail-closed pre-motion restock action coordinator",
            ),
            DeclareLaunchArgument(
                "autonomous_campaign",
                default_value="false",
                description=(
                    "Continuously survey front/back and drain all compatible RestockProduct "
                    "transfers after each survey cycle"
                ),
            ),
            DeclareLaunchArgument(
                "campaign_max_cycles",
                default_value="0",
                description="Autonomous survey/restock cycles; zero runs until shutdown",
            ),
            DeclareLaunchArgument(
                "campaign_restart_acknowledged",
                default_value="false",
                description=(
                    "Operator acknowledgment that the robot is verified settled (Card 086 "
                    "stage 1). While false the campaign reports PHASE_RECOVERING and sends no "
                    "goal; set it later with `ros2 param set /autonomous_restock_campaign "
                    "restart_acknowledged true`. Pass true only for a fresh simulated world"
                ),
            ),
            DeclareLaunchArgument(
                "campaign_cycle_period",
                default_value="30.0",
                description="Wall-clock idle delay between autonomous campaign cycles",
            ),
            DeclareLaunchArgument(
                "survey_viewpoint",
                default_value="false",
                description=(
                    "Start the wrist-camera survey action server, which aims "
                    "wrist_camera_optical_frame at a commanded viewpoint"
                ),
            ),
            DeclareLaunchArgument(
                "lane_survey",
                default_value="true",
                description=(
                    "Start the lane survey action server and the wrist-camera lane depletion "
                    "producer it acquires through, plus the evaluator that measures that producer "
                    "against simulator ground truth"
                ),
            ),
            DeclareLaunchArgument(
                "tray_survey",
                default_value="false",
                description=(
                    "Start the coordinated tray survey action server (survey_tray): overview "
                    "stations through /survey_viewpoint, then one confirmation viewpoint for "
                    "one selected candidate. Enabling it also enables survey_viewpoint; "
                    "autonomous_campaign starts it automatically because the mode loop's "
                    "SURVEY_TRAY path is this action. Read-only; needs "
                    "tray_overview_perception (and tray_confirm_perception for anything but "
                    "an overview-only result)"
                ),
            ),
            DeclareLaunchArgument(
                "tray_overview_perception",
                default_value="true",
                description=(
                    "Run wrist-camera tray overview perception on "
                    "/perception/tray_candidates (hypotheses only; the world state never "
                    "ingests this topic). On by default: the sensor-driven default's tray "
                    "evidence comes from the camera. Needs wrist_camera"
                ),
            ),
            DeclareLaunchArgument(
                "tray_confirm_perception",
                default_value="true",
                description=(
                    "Run wrist-camera tray confirm perception on "
                    "/perception/object_observations, the topic the world state admits. On by "
                    "default: it is the sensor-driven default's product source, which is why "
                    "object_observation_topic is demoted by default (one publisher per topic). "
                    "For the ground-truth path pin object_observation_topic back to "
                    "/perception/object_observations and turn this off"
                ),
            ),
            DeclareLaunchArgument(
                "lane_observation_topic",
                default_value="/perception/lane_observations",
                description=(
                    "Which lane-observation topic the world state ingests. Defaults to the "
                    "wrist camera survey's topic, the sensor-driven default; a survey is "
                    "required before the first transfer either way. Simulator ground truth "
                    "publishes on /perception/ground_truth/lane_observations — pin that "
                    "explicitly to run the ground-truth path"
                ),
            ),
            DeclareLaunchArgument(
                "planning_smoke",
                default_value="false",
                description="Run the finite planning acceptance client",
            ),
            DeclareLaunchArgument(
                "planning_timeout",
                default_value="5.0",
                description="Maximum OMPL planning time per smoke-test request",
            ),
            simulation,
            moveit,
            scene_projector,
            depth_obstacle_node,
            attachment_adapter_node,
            task_coordinator_node,
            survey_viewpoint_node,
            lane_survey_node,
            tray_survey_node,
            autonomous_campaign_node,
            smoke_client,
        ]
    )
