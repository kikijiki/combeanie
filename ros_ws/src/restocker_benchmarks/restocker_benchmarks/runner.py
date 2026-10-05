# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Drive one headless benchmark run and record what happened, including when it fails."""
# Not a pass/fail gate: failures are recorded for later comparison across runs.
#
# A failed goal does not stop the run (startup costs about a minute). Later goals are marked
# ``preceded_by_failure`` so the report can separate independent failures from ones that
# inherited a robot still holding a product.
#
# ``minimum_clearance_m`` on the action result is copied from a struct nothing assigns (always
# 0.0), so it is recorded as unavailable with the reason.

from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct
from restocker_interfaces.msg import RestockCoordinatorStatus
from restocker_interfaces.srv import GetWorldState
import yaml

from restocker_benchmarks import (  # noqa: I100,I101 - ament and Ruff disagree
    RECORD_SCHEMA_VERSION,
    taxonomy,
)
from restocker_benchmarks import scenarios as scenario_table  # noqa: I100

# A full transfer is six planned segments, two gripper moves and two attachment transactions.
# Kept above the coordinator's own total timeout so the coordinator reports an overrun.
GOAL_TIMEOUT_S = 450.0
# Gazebo, the controllers, MoveIt and the coordinator's startup authority probe, cold.
STARTUP_TIMEOUT_S = 300.0
# Scenario completeness, measured from the first admitted-telemetry snapshot: the fixture spawn
# is paced (one ``create`` per 0.25 s), so a 62-product dense scenario needs tens of seconds
# after the world state is up. Coordinator readiness fired 7-9 s into that spawn on the 2026-09-23
# dense campaign, before the addressed ``stock_*`` sources existed; dispatching then made every
# goal a ``product_unobserved`` harness terminal. Bounded so a fixture that never finishes is a
# classified ``startup_failed`` rather than a hang.
SCENARIO_READY_TIMEOUT_S = 180.0
# Time allowed for a clean launch shutdown before it is killed (Gazebo sometimes ignores SIGINT).
SHUTDOWN_GRACE_S = 45.0

# Fixed for every run; not scenario knobs. The system is benchmarked headless with motion enabled.
FIXED_LAUNCH_ARGUMENTS = {
    "gui": "false",
    "rviz": "false",
    "planning_smoke": "false",
    "motion_enabled": "true",
    "controller_timeout": "30.0",
    # Ground-truth path pins (Milestone 10 Stage 7 default switch): GT back on the ingest topic,
    # GT lane evidence, wrist tray duties off — one producer per world-state topic.
    "object_observation_topic": "/perception/object_observations",
    "lane_observation_topic": "/perception/ground_truth/lane_observations",
    "tray_overview_perception": "false",
    "tray_confirm_perception": "false",
}

# The shipped default of obstacle_perception in baseline.launch.py, read from the launch file the
# runner actually invokes so the recorded value cannot drift from the launched system.
OBSTACLE_PERCEPTION_DEFAULT_RE = re.compile(
    r'DeclareLaunchArgument\(\s*"obstacle_perception",\s*default_value="([^"]+)"'
)


def obstacle_perception_default_from(launch_text: str) -> str | None:
    """Return the declared default of obstacle_perception in a baseline launch file's source."""
    match = OBSTACLE_PERCEPTION_DEFAULT_RE.search(launch_text)
    return match.group(1) if match else None


def effective_obstacle_perception(
    launch_arguments: dict, launch_text: str | None = None
) -> str | None:
    """
    Return the obstacle_perception value the launched system will actually run with.

    An explicit launch argument wins; otherwise the value is the shipped default of the
    baseline launch file (read from the installed share the runner launches). Recorded per run
    so a campaign's provenance carries the pipeline the numbers were measured under — the
    default flipped to true with Card 023, and future flips must be visible in run records.
    """
    if "obstacle_perception" in launch_arguments:
        return str(launch_arguments["obstacle_perception"])
    if launch_text is None:
        try:
            launch_path = (
                Path(get_package_share_directory("restocker_bringup"))
                / "launch"
                / "baseline.launch.py"
            )
            launch_text = launch_path.read_text(encoding="utf-8")
        except Exception:  # noqa: BLE001 - provenance must never fail the run; None says "unknown"
            return None
    return obstacle_perception_default_from(launch_text)


def repository_root() -> Path:
    """Return the repository root, which is where results and provenance are anchored."""
    # The runner is installed into the workspace, so the source tree is not below it. ``just``
    # sets the environment variable; the fallback keeps a manual invocation working from a
    # source checkout.
    override = os.environ.get("RESTOCKER_REPOSITORY_ROOT")
    if override:
        return Path(override).resolve()
    return Path.cwd().resolve()


def _git(root: Path, *args: str) -> str | None:
    try:
        completed = subprocess.run(
            ["git", "-C", str(root), *args],
            check=True,
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return completed.stdout.strip()


def _file_digest(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _hardware_profile() -> dict[str, object]:
    """Record enough of the machine to explain a timing difference between two campaigns."""
    model = None
    try:
        for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("model name"):
                model = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    memory_kib = None
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("MemTotal:"):
                memory_kib = int(line.split()[1])
                break
    except (OSError, ValueError):
        pass
    return {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "cpu_model": model,
        "cpu_count": os.cpu_count(),
        "memory_total_kib": memory_kib,
        # Cheapest evidence of whether the benchmark ran alone or beside other agents.
        "load_average_at_start": os.getloadavg(),
        "gpu_at_start": _gpu_profile(),
    }


def _gpu_profile() -> dict[str, object]:
    """
    Record GPU utilisation and VRAM (amdgpu sysfs).

    Load average is blind to the GPU, and the world renders two RGB-D sensors through EGL, so
    frame rate differs between a busy and an idle GPU at the same load. Best-effort: a host with
    no amdgpu card records ``None``.
    """
    root = Path("/sys/class/drm")
    profile: dict[str, object] = {"card": None}
    try:
        cards = sorted(path for path in root.glob("card[0-9]*") if "-" not in path.name)
    except OSError:
        return profile
    for card in cards:
        device = card / "device"
        readings: dict[str, object] = {}
        for name, key in (
            ("gpu_busy_percent", "busy_percent"),
            ("mem_info_vram_used", "vram_used_bytes"),
            ("mem_info_vram_total", "vram_total_bytes"),
        ):
            try:
                readings[key] = int((device / name).read_text(encoding="utf-8").strip())
            except (OSError, ValueError):
                readings[key] = None
        if any(value is not None for value in readings.values()):
            profile = {"card": card.name, **readings}
            break
    return profile


def arm_identity() -> dict[str, object]:
    """
    Identify the manipulator the launch will spawn, for population gating.

    The label is the ``ur_description`` configuration directory named by ``arm.xacro`` —
    ``ur10e`` on the official description, absent on the retired custom arm's tree. A record
    that cannot name its arm records ``label: null`` and is never eligible for the UR10e
    campaign population, which is what keeps pre-swap or differently-armed results out of the
    rates. The xacro digest pins the exact description a run used.
    """
    try:
        path = Path(get_package_share_directory("restocker_description")) / "urdf" / "arm.xacro"
        text = path.read_text(encoding="utf-8")
    except (OSError, LookupError):
        return {"label": None, "arm_xacro_sha256": None, "source": None}
    match = re.search(r"ur_description\)/config/([a-z0-9_]+)/default_kinematics", text)
    return {
        "label": match.group(1) if match else None,
        "arm_xacro_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
        "source": str(path),
    }


def _planner_backend() -> dict[str, object]:
    """Read the planner identity out of the config the launch loads."""
    try:
        path = (
            Path(get_package_share_directory("restocker_moveit_config"))
            / "config"
            / "ompl_planning.yaml"
        )
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError, LookupError):
        return {"available": False}
    return {
        "available": True,
        "planning_plugins": document.get("planning_plugins"),
        "planner_configs": sorted(document.get("planner_configs", {})),
        "config_sha256": _file_digest(path),
    }


class BenchmarkClient(Node):
    """A ROS client that drives goals at the public action and records every terminal outcome."""

    def __init__(self) -> None:
        super().__init__("restock_benchmark_client")
        self.latest_status: RestockCoordinatorStatus | None = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            RestockCoordinatorStatus,
            "/restock_action_coordinator/status",
            self._on_status,
            status_qos,
        )
        self.action_client = ActionClient(self, RestockProduct, "/restock_product")
        self.world_state = self.create_client(GetWorldState, "/world_state/get_snapshot")

    def _on_status(self, status: RestockCoordinatorStatus) -> None:
        self.latest_status = status

    def spin_until(self, predicate, timeout_s: float, alive):
        """Spin until ``predicate`` returns a truthy value, the launch dies, or time runs out."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if not alive():
                return None
            found = predicate()
            if found:
                return found
        return None

    def snapshot(self, alive, timeout_s: float = 30.0):
        """Request one authoritative world-state snapshot."""
        if not self.world_state.wait_for_service(timeout_sec=timeout_s):
            return None
        future = self.world_state.call_async(GetWorldState.Request())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if not alive():
                return None
            if future.done():
                result = future.result()
                return None if result is None else result.snapshot
        return None

    def admitted_snapshot(self, alive):
        """Return a snapshot only once it carries admitted robot telemetry."""
        # Coordinator readiness does not imply admitted telemetry, and selection rejects a
        # snapshot with unset telemetry.
        snapshot = self.snapshot(alive)
        if snapshot is None:
            return None
        if snapshot.robot.telemetry_source_id and snapshot.robot.telemetry_revision > 0:
            return snapshot
        return None

    def fixture_snapshot(self, alive, stocked_ids):
        """
        Return a snapshot only once telemetry is admitted and every stocked product is in it.

        Scenario completeness, not coordinator readiness: products spawn one at a time, and the
        addressed ``stock_*`` sources of a dense fixture are spawned late enough that the
        coordinator can be ready (and telemetry admitted) while half the fixture is still
        missing. Dispatching before ``fixture_fully_observed`` is the race the 2026-09-23 dense
        campaign measured on all 24 runs.
        """
        snapshot = self.admitted_snapshot(alive)
        if snapshot is None:
            return None
        if fixture_fully_observed(snapshot, stocked_ids):
            return snapshot
        return None


def _harness_outcome(reason: str, detail: str) -> dict[str, object]:
    """Build the terminal record for something the harness observed, not the system."""
    return {
        "status": taxonomy.HARNESS_STATUS,
        "status_name": "HARNESS",
        # ``reason`` is the short stable key the taxonomy groups on; ``detail`` is the sentence.
        # Kept apart so the family column does not carry a paragraph.
        "reason": reason,
        "detail": detail,
        "detail_full": detail,
        "family": taxonomy.classify_family(taxonomy.HARNESS_STATUS, reason),
        "signature": taxonomy.signature(taxonomy.HARNESS_STATUS, reason),
    }


def stocked_ids_of(document: dict) -> set[str]:
    """Return the scenario document's stocked source-object ids (what must be observed)."""
    return {product["source_object_id"] for product in document.get("products", [])}


def fixture_fully_observed(snapshot, stocked_ids) -> bool:
    """
    Whether a world-state snapshot contains every scenario product (observed == stocked).

    Pure so the readiness gate is unit-testable without a simulator; a partial spawn — the
    dense campaign's 25-37 of 62 — must read False even though the snapshot itself is healthy.
    """
    if snapshot is None:
        return False
    observed = {tracked.source_object_id for tracked in snapshot.objects}
    return stocked_ids <= observed


def _occupied_lanes(snapshot) -> dict[str, list[int]]:
    return {lane.id: list(lane.contents) for lane in snapshot.lanes if lane.contents}


def _run_goal(client: BenchmarkClient, goal_spec, object_id, alive) -> dict[str, object]:
    """Send one goal, collect its feedback trace, and record its terminal outcome."""
    goal = RestockProduct.Goal()
    goal.has_object_id = object_id is not None
    goal.object_id = object_id if object_id is not None else 0
    goal.has_lane_id = goal_spec.lane_id is not None
    goal.lane_id = goal_spec.lane_id or ""

    trace: list[dict[str, object]] = []

    def on_feedback(message) -> None:
        feedback = message.feedback
        elapsed = feedback.elapsed_sim_time.sec + feedback.elapsed_sim_time.nanosec * 1e-9
        entry = {
            "state": int(feedback.state),
            "state_name": taxonomy.state_name(int(feedback.state)),
            "attempt": int(feedback.attempt),
            "recovery_attempt": int(feedback.recovery_attempt),
            "elapsed_sim_s": elapsed,
            "detail": feedback.detail,
        }
        # Feedback repeats while a state holds; only transitions are kept.
        if not trace or trace[-1]["state"] != entry["state"]:
            trace.append(entry)

    record: dict[str, object] = {
        "label": goal_spec.label,
        "addressed": goal_spec.source_object_id is not None,
        "requested_source_object_id": goal_spec.source_object_id,
        "requested_object_id": object_id,
        "requested_lane_id": goal_spec.lane_id,
        "trace": trace,
    }
    started = time.monotonic()

    goal_future = client.action_client.send_goal_async(goal, feedback_callback=on_feedback)
    handle = client.spin_until(lambda: goal_future.done() and goal_future.result(), 60.0, alive)
    if handle is None:
        record["wall_s"] = time.monotonic() - started
        record["outcome"] = _harness_outcome(
            "goal was never answered", "send_goal_async did not complete"
        )
        return record
    if not handle.accepted:
        record["wall_s"] = time.monotonic() - started
        record["outcome"] = _harness_outcome(
            "goal was rejected", "the coordinator refused to accept the goal"
        )
        return record

    result_future = handle.get_result_async()
    wrapped = client.spin_until(
        lambda: result_future.done() and result_future.result(), GOAL_TIMEOUT_S, alive
    )
    record["wall_s"] = time.monotonic() - started
    if wrapped is None:
        record["outcome"] = _harness_outcome(
            "no result before the harness deadline",
            f"the goal produced no result within {GOAL_TIMEOUT_S:g} s, or the launch exited",
        )
        return record

    result = wrapped.result
    elapsed_sim = result.elapsed_sim_time.sec + result.elapsed_sim_time.nanosec * 1e-9
    furthest = taxonomy.furthest_state([int(entry["state"]) for entry in trace])
    record["outcome"] = {
        "status": int(result.status),
        "status_name": taxonomy.STATUS_NAMES.get(int(result.status), f"STATUS_{result.status}"),
        "detail": result.detail,
        "detail_full": result.detail,
        "family": taxonomy.classify_family(int(result.status), result.detail),
        "signature": taxonomy.signature(int(result.status), result.detail),
    }
    record["attempt_count"] = int(result.attempt_count)
    record["retry_count"] = int(result.retry_count)
    record["recovery_attempts"] = max(
        (int(entry["recovery_attempt"]) for entry in trace), default=0
    )
    record["elapsed_sim_s"] = elapsed_sim
    record["initial_world_revision"] = int(result.initial_world_revision)
    record["final_world_revision"] = int(result.final_world_revision)
    record["furthest_state"] = furthest
    record["furthest_state_name"] = taxonomy.state_name(furthest)
    record["phase_sim_s"] = taxonomy.phase_durations(trace, elapsed_sim)
    # See the module comment: the result's clearance field is always 0.0.
    record["minimum_clearance_m"] = None
    record["minimum_clearance_unavailable_reason"] = (
        "RestockProduct.Result.minimum_clearance_m is copied from RestockTaskMetrics, which no "
        "production path assigns; it is 0.0 on every run. tools/diagnostics/collide.py computes "
        "real clearance against the workcell geometry from a recorded joint trace."
    )
    return record


def _settling_error(scenario_document: dict, snapshot) -> dict[str, object]:
    """Compare the observed product poses against the poses the scenario file fixed."""
    # Not a perception metric: the ground-truth adapter reads the simulator, so this measures how
    # far physics moved a product from its seeded pose before the run began.
    expected = {
        product["source_object_id"]: product["spawn_pose"]
        for product in scenario_document.get("products", [])
    }
    observed = {tracked.source_object_id: tracked for tracked in snapshot.objects}
    errors = []
    for source_object_id, spawn_pose in sorted(expected.items()):
        tracked = observed.get(source_object_id)
        if tracked is None:
            continue
        position = tracked.pose.pose.position
        errors.append(
            {
                "source_object_id": source_object_id,
                "translation_error_m": (
                    (position.x - spawn_pose[0]) ** 2
                    + (position.y - spawn_pose[1]) ** 2
                    + (position.z - spawn_pose[2]) ** 2
                )
                ** 0.5,
            }
        )
    return {
        "stocked": len(expected),
        "observed": sum(1 for key in expected if key in observed),
        "max_translation_error_m": max(
            (entry["translation_error_m"] for entry in errors), default=None
        ),
        "per_product": errors,
    }


def _terminate(process: subprocess.Popen) -> dict[str, object]:
    """Shut the launch down, escalating only as far as it makes us."""
    if process.poll() is not None:
        return {"method": "already_exited", "returncode": process.returncode}
    method = "sigint"
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
    except (OSError, ProcessLookupError):
        return {"method": "gone", "returncode": process.poll()}
    deadline = time.monotonic() + SHUTDOWN_GRACE_S
    while time.monotonic() < deadline and process.poll() is None:
        time.sleep(0.5)
    if process.poll() is None:
        method = "sigkill"
        with contextlib.suppress(OSError, ProcessLookupError):
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            method = "sigkill_timed_out"
    return {"method": method, "returncode": process.poll()}


def execute_run(scenario, seed: str, run_directory: Path, repository: Path) -> dict:
    """Launch the system once, drive the scenario's goals, and return the run record."""
    run_directory.mkdir(parents=True, exist_ok=True)
    stock_config = (
        Path(get_package_share_directory("restocker_gazebo")) / "config" / scenario.stock_config
    )
    launch_arguments = dict(FIXED_LAUNCH_ARGUMENTS)
    launch_arguments["scenario_config"] = str(stock_config)
    launch_arguments["scenario_seed"] = seed

    environment = dict(os.environ)
    # The generated scenario lands in the run directory so the document a failing run spawned is
    # archived beside its result.
    environment["RESTOCKER_GENERATED_SCENARIO_DIR"] = str(run_directory)

    console_path = run_directory / "console.log"
    command = [
        "ros2",
        "launch",
        "restocker_bringup",
        "baseline.launch.py",
        *(f"{key}:={value}" for key, value in launch_arguments.items()),
    ]

    record: dict[str, object] = {
        "schema_version": RECORD_SCHEMA_VERSION,
        "scenario": scenario.name,
        "scenario_description": scenario.description,
        "seed": seed,
        "run_directory": str(run_directory),
        "started_utc": dt.datetime.now(dt.UTC).isoformat(),
        "provenance": {
            "git_commit": _git(repository, "rev-parse", "HEAD"),
            "git_describe": _git(repository, "describe", "--always", "--dirty"),
            "git_dirty": bool(_git(repository, "status", "--porcelain")),
            "flake_lock_sha256": _file_digest(repository / "flake.lock"),
            "stock_config": str(stock_config),
            "stock_config_sha256": _file_digest(stock_config),
            "launch_arguments": launch_arguments,
            # The pipeline this run actually exercised: an explicit argument, else the shipped
            # baseline default (Card 023 flipped it to true). Campaigns must predeclare it.
            "obstacle_perception": effective_obstacle_perception(launch_arguments),
            "planner_backend": _planner_backend(),
            "model_backend": {
                # The deterministic baseline runs no learned model. Recording its absence keeps a
                # later campaign with a reasoner from being compared to this one by accident.
                "name": "none",
                "note": "deterministic baseline; ground-truth perception, no learned model",
            },
            "perception_backend": "gazebo_ground_truth",
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "gz_partition": os.environ.get("GZ_PARTITION"),
            # The arm gate: UR10e-only populations exclude anything that cannot name this arm.
            "arm": arm_identity(),
            "hardware": _hardware_profile(),
        },
        "goals": [],
    }

    with console_path.open("w", encoding="utf-8") as console:
        process = subprocess.Popen(  # noqa: S603 - fixed argv, no shell
            command,
            cwd=str(repository),
            env=environment,
            stdout=console,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        def alive() -> bool:
            return process.poll() is None

        # rclpy is initialized once in main; each run gets a fresh node on the same context.
        client = BenchmarkClient()
        try:
            _drive(client, scenario, run_directory, seed, record, alive)
        finally:
            client.destroy_node()
            record["shutdown"] = _terminate(process)

    # Load on the way out as well: a run beside other work and the same run alone are different
    # measurements, and the finish load says whether the neighbourhood changed mid-campaign.
    record["provenance"]["hardware"]["load_average_at_finish"] = list(os.getloadavg())
    record["finished_utc"] = dt.datetime.now(dt.UTC).isoformat()
    return record


def _spawned_scenario(run_directory: Path, seed: str, stock_config: Path) -> dict:
    """Return the scenario document that was spawned, archiving it beside the result."""
    if seed != "fixed":
        # simulation.launch.py writes the generated document into the directory the runner
        # pointed it at, named for the seed.
        generated = sorted(run_directory.glob("*.yaml"))
        if generated:
            return yaml.safe_load(generated[0].read_text(encoding="utf-8"))
    archived = run_directory / "scenario.yaml"
    shutil.copyfile(stock_config, archived)
    return yaml.safe_load(archived.read_text(encoding="utf-8"))


def _drive(client, scenario, run_directory: Path, seed: str, record: dict, alive) -> None:
    """Wait for readiness and scenario completeness, then drive every goal, recording each."""
    ready = client.spin_until(
        lambda: client.latest_status is not None and client.latest_status.admission_ready,
        STARTUP_TIMEOUT_S,
        alive,
    )
    if not ready:
        record["outcome"] = "startup_failed"
        record["outcome_detail"] = (
            "the coordinator never published admission readiness"
            if alive()
            else "the launch exited before the coordinator became ready"
        )
        return

    # The spawned scenario document is loaded before the completeness wait so the fixture a
    # timed-out run was waiting on is archived beside its record, not only the one that passed.
    stock_config = Path(record["provenance"]["stock_config"])
    document = _spawned_scenario(run_directory, seed, stock_config)
    record["stocked_products"] = [
        {
            "source_object_id": product["source_object_id"],
            "product_class": product["product_class"],
            "spawn_pose": product["spawn_pose"],
        }
        for product in document.get("products", [])
    ]
    record["scenario_provenance"] = document.get("generated_from")
    stocked_ids = stocked_ids_of(document)

    # Coordinator readiness plus admitted telemetry is not scenario completeness: dispatch only
    # when every scenario product has spawned and been observed (observed == stocked), and fail
    # closed as a classified startup failure if the fixture never gets there.
    before = client.spin_until(
        lambda: client.fixture_snapshot(alive, stocked_ids),
        SCENARIO_READY_TIMEOUT_S,
        alive,
    )
    if before is None:
        partial = client.admitted_snapshot(alive)
        if partial is None:
            record["ground_truth_observation"] = {
                "stocked": len(stocked_ids),
                "observed": 0,
                "max_translation_error_m": None,
                "per_product": [],
            }
            record["lanes_before"] = {}
            record["outcome"] = "startup_failed"
            record["outcome_detail"] = "the world state never admitted robot telemetry"
            return
        if fixture_fully_observed(partial, stocked_ids):
            # Completed between the last timed poll and this final check.
            before = partial
        else:
            observed = sum(
                1 for tracked in partial.objects if tracked.source_object_id in stocked_ids
            )
            record["ground_truth_observation"] = _settling_error(document, partial)
            record["lanes_before"] = _occupied_lanes(partial)
            record["outcome"] = "startup_failed"
            record["outcome_detail"] = (
                f"the scenario fixture never finished spawning: observed {observed} of "
                f"{len(stocked_ids)} stocked products within {SCENARIO_READY_TIMEOUT_S:g} s "
                "of admitted telemetry (readiness gate: observed == stocked before dispatch)"
            )
            return

    record["ground_truth_observation"] = _settling_error(document, before)
    record["lanes_before"] = _occupied_lanes(before)

    if not client.action_client.wait_for_server(timeout_sec=60.0):
        record["outcome"] = "startup_failed"
        record["outcome_detail"] = "the restock action server never became available"
        return

    observed = {tracked.source_object_id: tracked.id for tracked in before.objects}
    goals = scenario.goals
    if goals is None:
        # One unaddressed goal per stocked product: with a seed the product count varies, and the
        # coordinator chooses what to move and where.
        goals = tuple(
            scenario_table.Goal(f"unaddressed goal {index + 1}")
            for index in range(len(record["stocked_products"]))
        )
    record["scenario_goals"] = [asdict(goal) for goal in goals]

    failed_already = False
    for goal_spec in goals:
        object_id = None
        if goal_spec.source_object_id is not None:
            object_id = observed.get(goal_spec.source_object_id)
            if object_id is None:
                record["goals"].append(
                    {
                        "label": goal_spec.label,
                        "addressed": True,
                        "requested_source_object_id": goal_spec.source_object_id,
                        "preceded_by_failure": failed_already,
                        "outcome": _harness_outcome(
                            "scenario product was never observed",
                            f"{goal_spec.source_object_id} is not in the world state snapshot",
                        ),
                    }
                )
                failed_already = True
                continue
        goal_record = _run_goal(client, goal_spec, object_id, alive)
        goal_record["preceded_by_failure"] = failed_already
        after = client.snapshot(alive)
        goal_record["lanes_after"] = _occupied_lanes(after) if after is not None else None
        goal_record["robot_holds_object_after"] = (
            bool(after.robot.has_held_object) if after is not None else None
        )
        record["goals"].append(goal_record)
        if goal_record["outcome"]["status"] != taxonomy.STATUS_SUCCEEDED:
            failed_already = True
        if not alive():
            break

    final = client.snapshot(alive)
    record["lanes_after"] = _occupied_lanes(final) if final is not None else None
    record["robot_holds_object_at_end"] = (
        bool(final.robot.has_held_object) if final is not None else None
    )
    record["outcome"] = "completed" if alive() else "launch_exited"


def main(argv: list[str] | None = None) -> int:
    """Run one scenario, once per seed and repetition, writing a record for each run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", help="scenario name; see restocker_benchmarks.scenarios")
    parser.add_argument(
        "--seed",
        action="append",
        default=None,
        help="scenario seed, 'fixed' or an unsigned integer; repeat to sweep seeds",
    )
    parser.add_argument("--repeat", type=int, default=1, help="runs per seed")
    parser.add_argument(
        "--results-dir",
        default=None,
        help="campaign directory (default: <repository root>/benchmark-results)",
    )
    arguments = parser.parse_args(argv)

    scenario = scenario_table.scenario(arguments.scenario)
    seeds = arguments.seed or ["fixed"]
    if not scenario.seedable and any(seed != "fixed" for seed in seeds):
        raise SystemExit(
            f"scenario {scenario.name!r} fixes its stock poses; it takes no seed. Use "
            f"'seeded_autonomous' to vary the stock draw."
        )

    # Fail fast: without the planning profile the coordinator never becomes ready.
    try:
        get_package_share_directory("moveit_ros_move_group")
    except Exception:  # noqa: BLE001 - ament raises a package-specific error type
        raise SystemExit(
            "move_group is not on the search path; run benchmarks through 'just benchmark', "
            "which enters the baseline profile"
        ) from None

    repository = repository_root()
    results_root = (
        Path(arguments.results_dir) if arguments.results_dir else repository / "benchmark-results"
    )

    failures = 0
    rclpy.init()
    try:
        for seed in seeds:
            for repetition in range(arguments.repeat):
                stamp = dt.datetime.now(dt.UTC).strftime("%Y%m%dT%H%M%SZ")
                run_id = f"seed-{seed}-{stamp}-{repetition:02d}"
                run_directory = results_root / scenario.name / run_id
                print(f"[benchmark] {scenario.name} seed={seed} -> {run_directory}", flush=True)
                record = execute_run(scenario, seed, run_directory, repository)
                record["run_id"] = run_id
                (run_directory / "run.json").write_text(
                    json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8"
                )
                outcomes = [goal["outcome"]["status_name"] for goal in record["goals"]]
                print(f"[benchmark] {run_id}: {record['outcome']} goals={outcomes}", flush=True)
                if record["outcome"] != "completed" or any(
                    name != "SUCCEEDED" for name in outcomes
                ):
                    failures += 1
    finally:
        rclpy.shutdown()

    # A run that recorded a failure is a successful benchmark: the exit status reports whether the
    # harness worked, not whether the system passed. Exit 0 unless nothing was recorded at all.
    print(f"[benchmark] {len(seeds) * arguments.repeat} run(s), {failures} with a failed goal")
    return 0
