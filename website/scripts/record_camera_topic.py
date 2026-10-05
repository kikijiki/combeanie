#!/usr/bin/env python3
"""Record a Gazebo/ROS image topic to H.264 MP4 for a fixed duration."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, topic: str, output: str, seconds: float, fps: float) -> None:
        super().__init__("docs_camera_recorder")
        self._output = output
        self._seconds = seconds
        self._fps = fps
        self._ffmpeg: subprocess.Popen[bytes] | None = None
        self._frames = 0
        self._started: float | None = None
        self._done = False
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _ensure_ffmpeg(self, width: int, height: int, encoding: str) -> None:
        if self._ffmpeg is not None:
            return
        pix = {
            "rgb8": "rgb24",
            "bgr8": "bgr24",
            "rgba8": "rgba",
            "bgra8": "bgra",
        }.get(encoding)
        if pix is None:
            raise RuntimeError(f"unsupported encoding {encoding}")
        cmd = [
            "ffmpeg",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            pix,
            "-s",
            f"{width}x{height}",
            "-r",
            str(self._fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "23",
            "-preset",
            "medium",
            "-movflags",
            "+faststart",
            self._output,
        ]
        self.get_logger().info("starting ffmpeg: " + " ".join(cmd))
        self._ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self._started = time.monotonic()

    def _on_image(self, msg: Image) -> None:
        if self._done:
            return
        self._ensure_ffmpeg(msg.width, msg.height, msg.encoding)
        assert self._ffmpeg is not None and self._ffmpeg.stdin is not None
        size = msg.step * msg.height if msg.step else len(msg.data)
        self._ffmpeg.stdin.write(bytes(msg.data[:size]))
        self._frames += 1
        assert self._started is not None
        if time.monotonic() - self._started >= self._seconds:
            self._done = True
            self.get_logger().info(
                f"duration reached after {self._frames} frames; shutting down"
            )
            raise SystemExit(0)

    def close(self) -> None:
        if self._ffmpeg is None:
            return
        if self._ffmpeg.stdin is not None:
            self._ffmpeg.stdin.close()
        self._ffmpeg.wait(timeout=60)
        self.get_logger().info(f"closed after {self._frames} frames -> {self._output}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--topic", default="/docs_side_camera/image")
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--fps", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args.topic, args.output, args.seconds, args.fps)
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node._frames > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
