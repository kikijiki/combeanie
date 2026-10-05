#!/usr/bin/env python3
"""Encode dump_image_frames.py output into an MP4."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame_dir", type=Path)
    parser.add_argument("out_mp4")
    parser.add_argument("--fps", type=float, default=10.0)
    args = parser.parse_args()
    frame_dir = args.frame_dir
    meta = {}
    for line in (frame_dir / "meta.txt").read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            meta[k] = v
    encoding = meta.get("encoding", "rgb8")
    width = int(meta["width"])
    height = int(meta["height"])
    pix = {
        "rgb8": "rgb24",
        "bgr8": "bgr24",
        "rgba8": "rgba",
        "bgra8": "bgra",
    }.get(encoding)
    if pix is None:
        raise SystemExit(f"unsupported encoding {encoding}")

    frames = sorted(frame_dir.glob("frame_*.raw"))
    if not frames:
        raise SystemExit("no frames")

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
        str(args.fps),
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
        args.out_mp4,
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    for path in frames:
        proc.stdin.write(path.read_bytes())
    proc.stdin.close()
    proc.wait(timeout=180)
    print(f"encoded {len(frames)} frames at {args.fps:g} fps -> {args.out_mp4}")
    return 0 if proc.returncode == 0 else proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
