#!/usr/bin/env python3
"""Measure delivered camera rates for one configuration, inside a leased ROS domain.

Emits one JSON object on stdout.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

TOPIC_TYPES = {
    "/overhead_camera/image": Image,
    "/overhead_camera/depth_image": Image,
    "/overhead_camera/camera_info": CameraInfo,
    "/wrist_camera/image": Image,
    "/wrist_camera/depth_image": Image,
    "/wrist_camera/camera_info": CameraInfo,
}


def loadavg1() -> float:
    with open("/proc/loadavg", encoding="utf-8") as handle:
        return float(handle.read().split()[0])


def gpu_busy() -> int | None:
    """Integrated-GPU utilisation percent (not visible in load average)."""
    try:
        with open("/sys/class/drm/card1/device/gpu_busy_percent", encoding="utf-8") as handle:
            return int(handle.read().strip())
    except (OSError, ValueError):
        return None


def vram_used_mib() -> int | None:
    try:
        with open("/sys/class/drm/card1/device/mem_info_vram_used", encoding="utf-8") as handle:
            return int(handle.read().strip()) // (1024 * 1024)
    except (OSError, ValueError):
        return None


def drm_engine_nanoseconds() -> dict[str, dict[str, int]]:
    """Per-process GPU engine time, keyed by "<pid>:<executable>".

    amdgpu exports drm-engine-gfx / drm-engine-compute in /proc/<pid>/fdinfo/<fd>. Differencing
    across the measurement window gives GPU time per process. A duplicated fd repeats the same
    descriptor, so each drm-client-id is counted once.
    """
    result: dict[str, dict[str, int]] = {}
    own = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == own:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as handle:
                parts = handle.read().split(b"\0")
        except OSError:
            continue
        if not parts or not parts[0]:
            continue
        executable = os.path.basename(parts[0].decode("utf-8", "replace"))
        engines: dict[str, int] = {}
        seen_clients: set[str] = set()
        directory = f"/proc/{pid}/fdinfo"
        try:
            descriptors = os.listdir(directory)
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                with open(f"{directory}/{descriptor}", encoding="utf-8") as handle:
                    fields = dict(
                        (line.split(":", 1)[0], line.split(":", 1)[1].strip())
                        for line in handle
                        if ":" in line
                    )
            except (OSError, ValueError):
                continue
            if fields.get("drm-driver") != "amdgpu":
                continue
            client = fields.get("drm-client-id")
            if client is None or client in seen_clients:
                continue
            seen_clients.add(client)
            for key, value in fields.items():
                if key.startswith("drm-engine-"):
                    with contextlib.suppress(ValueError, IndexError):
                        engines[key] = engines.get(key, 0) + int(value.split()[0])
        if engines:
            result[f"{pid}:{executable}"] = engines
    return result


def engine_delta(
    before: dict[str, dict[str, int]], after: dict[str, dict[str, int]]
) -> dict[str, dict[str, float]]:
    """Seconds of each GPU engine consumed per process between two samples."""
    delta: dict[str, dict[str, float]] = {}
    for key, engines in after.items():
        previous = before.get(key, {})
        row = {}
        for engine, value in engines.items():
            spent = (value - previous.get(engine, value)) / 1e9
            if spent > 0.01:
                row[engine] = round(spent, 3)
        if row:
            delta[key] = row
    return delta


def gz_sim_servers() -> list[str]:
    """Return one entry per live Gazebo server, as "<pid> <worktree>".

    Read from /proc, not `pgrep -f 'gz sim'`, which matches the shell that runs it. The launcher
    passes the whole command as a single argv element, so the argument vector is flattened and
    tokenised on whitespace and the first two tokens must be `gz` and `sim`. That cannot match
    the searching process.

    The worktree is recovered from the world path, since simulators sharing the GPU usually
    belong to another run.
    """
    servers = []
    own = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == own:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as handle:
                flattened = handle.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        tokens = flattened.split()
        if len(tokens) < 2 or os.path.basename(tokens[0]) != "gz" or tokens[1] != "sim":
            continue
        worktree = "?"
        for token in tokens:
            if "/worktrees/" in token:
                worktree = token.split("/worktrees/", 1)[1].split("/", 1)[0]
                break
        servers.append(f"{pid} {worktree}")
    return servers


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--repo", required=True)
    parser.add_argument("--logdir", required=True)
    parser.add_argument("--window", type=float, default=20.0)
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--first-message-timeout", type=float, default=240.0)
    parser.add_argument("--expect-wrist", type=int, default=1)
    parser.add_argument(
        "--qos",
        choices=("sensor_data", "reliable"),
        default="sensor_data",
        help=(
            "'sensor_data' is best-effort depth 5, which is what test_camera_runtime uses; "
            "'reliable' asks the transport to retransmit instead of dropping"
        ),
    )
    parser.add_argument(
        "--topics",
        default="",
        help=(
            "comma-separated subset to subscribe to. The default subscribes to all six, which "
            "is what test_camera_runtime does; a subset says how much of the loss is the "
            "subscriber's own"
        ),
    )
    parser.add_argument("--extra", default="{}")
    parser.add_argument(
        "--composition",
        choices=("gazebo", "bringup"),
        default="gazebo",
        help="'gazebo' is the simulator alone; 'bringup' is what test_camera_runtime launches",
    )
    arguments = parser.parse_args()

    subscribed = list(TOPIC_TYPES)
    if arguments.topics:
        requested = [name.strip() for name in arguments.topics.split(",") if name.strip()]
        unknown = [name for name in requested if name not in TOPIC_TYPES]
        if unknown:
            raise SystemExit(f"error: unknown topic(s): {unknown}")
        subscribed = requested
    expected = list(subscribed)
    if not arguments.expect_wrist:
        expected = [topic for topic in expected if not topic.startswith("/wrist")]

    record: dict = {
        "label": arguments.label,
        "subscribed_topics": subscribed,
        "subscriber_qos": arguments.qos,
        "extra": json.loads(arguments.extra),
        "domain": os.environ.get("ROS_DOMAIN_ID"),
        "load1_at_start": loadavg1(),
        "gpu_busy_at_start": gpu_busy(),
        "vram_used_mib_at_start": vram_used_mib(),
        "gz_servers_at_start": gz_sim_servers(),
        "started_epoch": time.time(),
    }

    launch_log = open(  # noqa: SIM115
        f"{arguments.logdir}/{arguments.label}.launch.log", "w", encoding="utf-8"
    )
    if arguments.composition == "bringup":
        # Same launch arguments as test_camera_runtime.
        launch_arguments = [
            "restocker_bringup",
            "simulation.launch.py",
            "gui:=false",
            "cameras:=true",
            "ground_truth:=true",
            "perception:=false",
            "controller_timeout:=30.0",
        ]
    else:
        launch_arguments = [
            "restocker_gazebo",
            "simulation.launch.py",
            "gui:=false",
            "cameras:=true",
            "ground_truth:=false",
            "spawn_scenario:=true",
            "controller_timeout:=60.0",
        ]
    record["composition"] = arguments.composition
    process = subprocess.Popen(
        [
            f"{arguments.repo}/scripts/with_workspace.bash",
            "ros2",
            "launch",
            *launch_arguments,
        ],
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    rclpy.init()
    node = rclpy.create_node("camera_rate_probe")
    # (wall_monotonic_arrival, sim_stamp_seconds) per topic.
    samples: dict[str, list[tuple[float, float]]] = {topic: [] for topic in TOPIC_TYPES}
    # Delivered geometry, to show whether a sweep point took effect.
    geometry: dict[str, str] = {}
    subscriptions = []

    def recorder(topic):
        def record_message(message):
            stamp = message.header.stamp
            samples[topic].append((time.monotonic(), stamp.sec + stamp.nanosec * 1e-9))
            if topic not in geometry:
                width = getattr(message, "width", None)
                height = getattr(message, "height", None)
                step = getattr(message, "step", None)
                geometry[topic] = f"{width}x{height}" + (f" step={step}" if step else "")

        return record_message

    subscription_qos = (
        qos_profile_sensor_data
        if arguments.qos == "sensor_data"
        else QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
    )
    for topic in subscribed:
        message_type = TOPIC_TYPES[topic]
        subscriptions.append(
            node.create_subscription(message_type, topic, recorder(topic), subscription_qos)
        )

    try:
        deadline = time.monotonic() + arguments.first_message_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(samples[topic] for topic in expected):
                break
        record["all_first_messages_arrived"] = all(samples[topic] for topic in expected)
        record["seconds_to_first_messages"] = time.monotonic() - (
            deadline - arguments.first_message_timeout
        )

        if record["all_first_messages_arrived"]:
            # Let the renderer reach steady state before the measured window opens.
            warm_end = time.monotonic() + arguments.warmup
            while time.monotonic() < warm_end:
                rclpy.spin_once(node, timeout_sec=0.1)

            record["load1_at_window"] = loadavg1()
            for topic in samples:
                samples[topic].clear()
            window_start = time.monotonic()
            window_end = window_start + arguments.window
            engines_before = drm_engine_nanoseconds()
            gpu_series: list[int] = []
            vram_series: list[int] = []
            next_sample = window_start
            while time.monotonic() < window_end:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now >= next_sample:
                    busy = gpu_busy()
                    if busy is not None:
                        gpu_series.append(busy)
                    vram = vram_used_mib()
                    if vram is not None:
                        vram_series.append(vram)
                    next_sample = now + 1.0
            record["window_wall_seconds"] = time.monotonic() - window_start
            record["gpu_engine_seconds_in_window"] = engine_delta(
                engines_before, drm_engine_nanoseconds()
            )
            record["gpu_busy_window"] = gpu_series
            record["gpu_busy_window_mean"] = (
                sum(gpu_series) / len(gpu_series) if gpu_series else None
            )
            record["vram_used_mib_window_max"] = max(vram_series) if vram_series else None

        record["load1_at_end"] = loadavg1()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        with contextlib.suppress(OSError):
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(OSError):
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=30)
        launch_log.close()

    topics_out = {}
    for topic, series in samples.items():
        entry: dict = {"count": len(series)}
        if len(series) >= 2:
            sim = [stamp for _wall, stamp in series]
            wall = [arrival for arrival, _stamp in series]
            sim_span = sim[-1] - sim[0]
            wall_span = wall[-1] - wall[0]
            entry["sim_span"] = sim_span
            entry["wall_span"] = wall_span
            # Same metric as the test: (n - 1) / simulation-time span.
            entry["rate_sim_hz"] = (len(series) - 1) / sim_span if sim_span > 0 else None
            entry["rate_wall_hz"] = (len(series) - 1) / wall_span if wall_span > 0 else None
            entry["rtf_estimate"] = sim_span / wall_span if wall_span > 0 else None
            deltas = sorted(sim[index + 1] - sim[index] for index in range(len(sim) - 1))
            entry["sim_delta_min"] = deltas[0]
            entry["sim_delta_median"] = deltas[len(deltas) // 2]
            entry["sim_delta_max"] = deltas[-1]
            entry["sim_deltas"] = [round(value, 6) for value in deltas]
        topics_out[topic] = entry
    record["topics"] = topics_out
    record["delivered_geometry"] = geometry
    record["gz_servers_at_end"] = gz_sim_servers()

    print(json.dumps(record))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
