#!/usr/bin/env python3
"""Subscribe to an image topic and write JPEG frames until SIGINT/timeout."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FrameDump(Node):
    def __init__(self, topic: str, out_dir: Path, every_n: int) -> None:
        super().__init__("docs_frame_dump")
        self._out_dir = out_dir
        self._every_n = max(1, every_n)
        self._count = 0
        self._saved = 0
        self._encoding = None
        self._width = 0
        self._height = 0
        self._done = False
        out_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        if self._done:
            return
        self._count += 1
        if self._count % self._every_n != 0:
            return
        self._encoding = msg.encoding
        self._width = msg.width
        self._height = msg.height
        # Raw bytes; encoding and size are stored in a sidecar.
        path = self._out_dir / f"frame_{self._saved:06d}.raw"
        size = msg.step * msg.height if msg.step else len(msg.data)
        path.write_bytes(bytes(msg.data[:size]))
        self._saved += 1
        if self._saved % 30 == 0:
            self.get_logger().info(f"saved {self._saved} frames")

    def stop(self) -> None:
        self._done = True
        meta = self._out_dir / "meta.txt"
        meta.write_text(
            f"encoding={self._encoding}\nwidth={self._width}\nheight={self._height}\nsaved={self._saved}\n"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic")
    parser.add_argument("out_dir")
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--every-n", type=int, default=2, help="keep every Nth frame")
    args = parser.parse_args()

    rclpy.init()
    node = FrameDump(args.topic, Path(args.out_dir), args.every_n)
    stop = False

    def _sig(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    start = time.monotonic()
    try:
        while rclpy.ok() and not stop and (time.monotonic() - start) < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"saved={node._saved} encoding={node._encoding}")
    return 0 if node._saved > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
