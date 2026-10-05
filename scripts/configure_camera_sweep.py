#!/usr/bin/env python3
"""Rewrite the installed camera description for one sweep point.

Edits land only in ros_ws/install, not the source tree. Each point starts from a pristine copy,
and the sweep ends with `--restore`.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

SHARE = "ros_ws/install/restocker_description/share/restocker_description/urdf"
WRIST_FILE = "sensors.xacro"
OVERHEAD_FILE = "workcell.xacro"


def pristine(path: Path) -> str:
    backup = path.with_suffix(path.suffix + ".pristine")
    if not backup.exists():
        backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return backup.read_text(encoding="utf-8")


def substitute_one(text: str, pattern: str, replacement: str, what: str) -> str:
    result, count = re.subn(pattern, replacement, text)
    if count != 1:
        raise SystemExit(f"error: {what} matched {count} times, expected exactly 1")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--overhead-rate", type=int)
    parser.add_argument("--overhead-size", help="WxH")
    parser.add_argument("--wrist-rate", type=int)
    parser.add_argument("--wrist-size", help="WxH")
    parser.add_argument("--wrist", choices=("present", "absent"), default="present")
    parser.add_argument("--restore", action="store_true")
    arguments = parser.parse_args()

    root = Path(arguments.repo) / SHARE
    wrist_path = root / WRIST_FILE
    overhead_path = root / OVERHEAD_FILE

    if arguments.restore:
        for path in (wrist_path, overhead_path):
            backup = path.with_suffix(path.suffix + ".pristine")
            if backup.exists():
                path.write_text(backup.read_text(encoding="utf-8"), encoding="utf-8")
                backup.unlink()
        print("restored pristine installed descriptions")
        return 0

    overhead = pristine(overhead_path)
    wrist = pristine(wrist_path)

    if arguments.overhead_rate is not None:
        overhead = substitute_one(
            overhead,
            r"<update_rate>6</update_rate>",
            f"<update_rate>{arguments.overhead_rate}</update_rate>",
            "overhead update_rate",
        )
    if arguments.overhead_size:
        width, height = arguments.overhead_size.split("x")
        overhead = substitute_one(
            overhead,
            r"<width>960</width><height>720</height>",
            f"<width>{width}</width><height>{height}</height>",
            "overhead image size",
        )

    if arguments.wrist == "absent":
        # Drop the whole <gazebo reference="...wrist_camera_link"> element.
        wrist, count = re.subn(
            r'\n    <gazebo reference="\$\{prefix\}wrist_camera_link">.*?\n    </gazebo>',
            "",
            wrist,
            flags=re.DOTALL,
        )
        if count != 1:
            raise SystemExit(f"error: wrist sensor block matched {count} times, expected 1")
    else:
        if arguments.wrist_rate is not None:
            wrist = substitute_one(
                wrist,
                r"<update_rate>5</update_rate>",
                f"<update_rate>{arguments.wrist_rate}</update_rate>",
                "wrist update_rate",
            )
        if arguments.wrist_size:
            width, height = arguments.wrist_size.split("x")
            wrist = substitute_one(
                wrist,
                r"<width>1280</width><height>720</height>",
                f"<width>{width}</width><height>{height}</height>",
                "wrist image size",
            )

    overhead_path.write_text(overhead, encoding="utf-8")
    wrist_path.write_text(wrist, encoding="utf-8")
    print(
        f"configured: overhead rate={arguments.overhead_rate} size={arguments.overhead_size} "
        f"wrist={arguments.wrist} rate={arguments.wrist_rate} size={arguments.wrist_size}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
