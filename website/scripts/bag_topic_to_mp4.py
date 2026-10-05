#!/usr/bin/env python3
"""Convert a rosbag2 sensor_msgs/Image topic into an H.264 MP4 via ffmpeg."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image


def main() -> int:
    if len(sys.argv) != 4:
        print(f"usage: {sys.argv[0]} bag_dir topic out.mp4", file=sys.stderr)
        return 2
    bag_dir, topic, out = sys.argv[1], sys.argv[2], sys.argv[3]

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(Path(bag_dir)), storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        # Exact match failed; try unique suffix match.
        matches = [name for name in topics if name.endswith(topic.lstrip("/")) or name == topic]
        if len(matches) != 1:
            raise SystemExit(f"topic {topic!r} not in bag; have {sorted(topics)}")
        topic = matches[0]

    ffmpeg: subprocess.Popen[bytes] | None = None
    frames = 0
    try:
        while reader.has_next():
            name, data, _timestamp = reader.read_next()
            if name != topic:
                continue
            msg = deserialize_message(data, Image)
            if ffmpeg is None:
                pix = {
                    "rgb8": "rgb24",
                    "bgr8": "bgr24",
                    "rgba8": "rgba",
                    "bgra8": "bgra",
                }.get(msg.encoding)
                if pix is None:
                    raise SystemExit(f"unsupported encoding {msg.encoding}")
                cmd = [
                    "ffmpeg",
                    "-y",
                    "-f",
                    "rawvideo",
                    "-pix_fmt",
                    pix,
                    "-s",
                    f"{msg.width}x{msg.height}",
                    "-r",
                    "12",
                    "-i",
                    "-",
                    "-an",
                    "-c:v",
                    "libx264",
                    "-pix_fmt",
                    "yuv420p",
                    "-crf",
                    "22",
                    "-preset",
                    "medium",
                    "-movflags",
                    "+faststart",
                    out,
                ]
                ffmpeg = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            assert ffmpeg.stdin is not None
            size = msg.step * msg.height if msg.step else len(msg.data)
            ffmpeg.stdin.write(bytes(msg.data[:size]))
            frames += 1
    finally:
        if ffmpeg is not None and ffmpeg.stdin is not None:
            ffmpeg.stdin.close()
            ffmpeg.wait(timeout=120)

    if frames == 0:
        raise SystemExit("no frames decoded")
    print(f"wrote {frames} frames -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
