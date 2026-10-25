#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Cut the recorded frames of a showcase run into the site's video, poster and loop.

showcase_edit.py FRAME_DIR SHOT_LOG OUT_DIR [--poster-frame N] [--fps 24]

FRAME_DIR holds every frame the recording camera saved (6 per simulated second); SHOT_LOG is the
director's record of which shot each frame belongs to. The edit keeps every frame of the close-up
shots, thins the wide and mid shots (the surveys and the carries are slow to watch), and encodes:

    restocking-showcase.mp4          1280x720, H.264, no audio
    restocking-showcase-poster.png   1280x720, the middle of the first shelf close-up
    restocking-showcase-loop.webp    640x360, a seam-matched loop of the wide shot

The loop is the pair of wide-shot frames, a few seconds apart, whose images differ least, so the
jump back to the start is as small as the scene allows; when no pair is close, the loop plays
forwards then backwards, which has no seam at all.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

# Frames of each shot kept per frame shown: the recording is 6 frames per simulated second, played
# at --fps, so stride 1 at 24 fps is four times real time.
STRIDE = {"wide": 2, "approach": 2, "carry": 2, "pick": 1, "place": 1}
# The camera pose takes effect a few frames after the director's call.
CUT_LATENCY_FRAMES = 3
LOOP_SECONDS = (4.0, 8.0)
LOOP_FPS = 10
# Mean absolute grey difference (0..255) under which two frames are accepted as a seam.
SEAM_LIMIT = 2.5


def frame_index(path: Path) -> int:
    return int(re.search(r"camera_(\d+)\.png$", path.name).group(1))


def load_shots(log_path: Path) -> list[tuple[int, str]]:
    cuts = []
    for line in log_path.read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        cuts.append((int(record["frame"]) + CUT_LATENCY_FRAMES, record["shot"]))
    return cuts


def shot_of(cuts: list[tuple[int, str]], index: int) -> str:
    shot = cuts[0][1]
    for start, name in cuts:
        if start <= index:
            shot = name
    return shot


def select(frames: list[Path], cuts: list[tuple[int, str]]) -> list[tuple[Path, str]]:
    """Every frame the edit shows, with its shot; strides count within an unbroken shot."""
    selected = []
    run_start = 0
    previous = None
    for path in frames:
        index = frame_index(path)
        shot = shot_of(cuts, index)
        if shot != previous:
            run_start = index
            previous = shot
        if (index - run_start) % STRIDE.get(shot, 1) == 0:
            selected.append((path, shot))
    return selected


def encode(selected: list[tuple[Path, str]], out_mp4: Path, fps: int) -> None:
    with tempfile.TemporaryDirectory() as scratch:
        for number, (path, _) in enumerate(selected):
            (Path(scratch) / f"{number:06d}.png").symlink_to(path.resolve())
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(fps),
                "-i",
                f"{scratch}/%06d.png",
                "-vf",
                "scale=1280:720:flags=lanczos",
                "-c:v",
                "libx264",
                "-crf",
                "22",
                "-preset",
                "slow",
                "-pix_fmt",
                "yuv420p",
                "-an",
                "-movflags",
                "+faststart",
                str(out_mp4),
            ],
            check=True,
        )


def grey_thumb(path: Path) -> np.ndarray:
    image = Image.open(path).convert("L").resize((192, 108), Image.BILINEAR)
    return np.asarray(image, dtype=np.float32)


def find_loop(wide: list[Path]) -> tuple[int, int, float]:
    """Indices (into ``wide``) of the closest pair of frames LOOP_SECONDS apart, and their gap."""
    thumbs = [grey_thumb(path) for path in wide]
    lowest, highest = (int(seconds * 6) for seconds in LOOP_SECONDS)
    best = (0, lowest, float("inf"))
    for start in range(len(thumbs) - lowest):
        for end in range(start + lowest, min(len(thumbs), start + highest + 1)):
            if frame_index(wide[end]) - frame_index(wide[start]) != end - start:
                continue  # the pair straddles a cut
            gap = float(np.abs(thumbs[start] - thumbs[end]).mean())
            if gap < best[2]:
                best = (start, end, gap)
    return best


def write_loop(wide: list[Path], out_webp: Path) -> str:
    if len(wide) < int(LOOP_SECONDS[0] * 6) + 2:
        return "no wide run long enough"
    start, end, gap = find_loop(wide)
    chosen = wide[start : end + 1]
    seamless = gap <= SEAM_LIMIT
    if not seamless:
        chosen = chosen + chosen[-2:0:-1]  # forwards, then backwards: no seam
    with tempfile.TemporaryDirectory() as scratch:
        for number, path in enumerate(chosen):
            (Path(scratch) / f"{number:05d}.png").symlink_to(path.resolve())
        # The loop plays 2 frames of the recording per output frame pair at LOOP_FPS: 6 Hz in,
        # LOOP_FPS out, so a little under twice real time.
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(LOOP_FPS),
                "-i",
                f"{scratch}/%05d.png",
                "-vf",
                "scale=640:360:flags=lanczos",
                "-loop",
                "0",
                "-quality",
                "70",
                "-an",
                str(out_webp),
            ],
            check=True,
        )
    kind = "seam gap" if seamless else "boomerang (no pair closer than the limit), gap"
    return f"loop frames {frame_index(wide[start])}..{frame_index(wide[end])}, {kind} {gap:.2f}"


def write_poster(selected: list[tuple[Path, str]], out_png: Path, frame: int | None) -> str:
    if frame is not None:
        source = next(path for path, _ in selected if frame_index(path) >= frame)
    else:
        place = [path for path, shot in selected if shot == "place"]
        source = place[len(place) // 2] if place else selected[len(selected) // 2][0]
    Image.open(source).convert("RGB").resize((1280, 720), Image.LANCZOS).save(out_png)
    return f"poster {source.name}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("frame_dir", type=Path)
    parser.add_argument("shot_log", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--poster-frame", type=int, default=None)
    parser.add_argument("--fps", type=int, default=24)
    arguments = parser.parse_args()
    frames = sorted(arguments.frame_dir.glob("*.png"), key=frame_index)
    cuts = load_shots(arguments.shot_log)
    if not frames or not cuts:
        print("no frames or no shot log", file=sys.stderr)
        return 2
    arguments.out_dir.mkdir(parents=True, exist_ok=True)
    selected = select(frames, cuts)
    encode(selected, arguments.out_dir / "restocking-showcase.mp4", arguments.fps)
    print(f"{len(frames)} frames recorded, {len(selected)} shown")
    print(
        write_poster(
            selected, arguments.out_dir / "restocking-showcase-poster.png", arguments.poster_frame
        )
    )
    wide = [path for path in frames if shot_of(cuts, frame_index(path)) == "wide"]
    print(write_loop(wide, arguments.out_dir / "restocking-showcase-loop.webp"))
    shutil.rmtree(arguments.out_dir / "__pycache__", ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
