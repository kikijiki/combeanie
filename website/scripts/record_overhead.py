#!/usr/bin/env python3
"""Record /overhead_camera/image to an H.264 MP4 via ffmpeg stdin."""

from __future__ import annotations

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, output: str) -> None:
        super().__init__("overhead_camera_recorder")
        self._output = output
        self._ffmpeg: subprocess.Popen[bytes] | None = None
        self._frames = 0
        self._started = time.monotonic()
        self.create_subscription(
            Image, "/overhead_camera/image", self._on_image, qos_profile_sensor_data
        )

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
            "15",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "26",
            "-preset",
            "veryfast",
            "-movflags",
            "+faststart",
            self._output,
        ]
        self.get_logger().info("starting ffmpeg: " + " ".join(cmd))
        self._ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def _on_image(self, msg: Image) -> None:
        self._ensure_ffmpeg(msg.width, msg.height, msg.encoding)
        assert self._ffmpeg is not None and self._ffmpeg.stdin is not None
        expected = msg.width * msg.height * (len(msg.data) // max(msg.width * msg.height, 1))
        # Prefer step-based size when available.
        size = msg.step * msg.height if msg.step else len(msg.data)
        self._ffmpeg.stdin.write(bytes(msg.data[:size]))
        self._frames += 1
        if self._frames % 30 == 0:
            self.get_logger().info(f"wrote {self._frames} frames")

    def close(self) -> None:
        if self._ffmpeg is None:
            return
        if self._ffmpeg.stdin is not None:
            self._ffmpeg.stdin.close()
        self._ffmpeg.wait(timeout=30)
        elapsed = time.monotonic() - self._started
        self.get_logger().info(f"closed after {self._frames} frames in {elapsed:.1f}s")


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} output.mp4", file=sys.stderr)
        return 2
    output = sys.argv[1]
    rclpy.init()
    node = Recorder(output)
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except Exception:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
