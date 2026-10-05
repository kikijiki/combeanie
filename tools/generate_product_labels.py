#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Regenerate the fictional product label textures shipped with restocker_gazebo.

The labels are original work drawn from flat rectangles and a hand-coded block font, so they
carry no third-party licence and depict no real commercial branding. They exist to give two
products that share a shape something a camera can tell apart.

The two can labels are identical except for the word, so colour histogram, mean hue or dominant
channel cannot separate them; the word is the only signal.

Run it with `python3 tools/generate_product_labels.py`; it rewrites the PNGs in place and prints
nothing unless a byte changed. It needs only the standard library.
"""

from __future__ import annotations

from pathlib import Path
import struct
import zlib

# restocker_gazebo owns the textures because it owns the product description that names them.
TEXTURE_DIR = (
    Path(__file__).resolve().parents[1]
    / "ros_ws"
    / "src"
    / "restocker_gazebo"
    / "materials"
    / "textures"
)

# A Gazebo cylinder's lateral surface is unwrapped with u once around the circumference and v
# along the axis, so width must tile and height is the product's full height.
WIDTH = 1024
HEIGHT = 512

# Three repetitions put one whole word toward the camera from any yaw. Two leave a seam edge-on
# over a wide range of angles; four halve the glyph width.
REPEATS = 3

FIELD = (196, 58, 46)
BAND = (242, 238, 230)
INK = (38, 34, 32)
RULE = (150, 38, 30)

BAND_TOP = 152
BAND_BOTTOM = 360
RULE_ROWS = ((44, 68), (444, 468))

GLYPH_WIDTH = 5
GLYPH_HEIGHT = 7
GLYPH_SCALE = 7
GLYPH_GAP = 1

# 5x7 block font covering the letters the shipped labels use. A missing glyph is an error, not a
# blank.
FONT = {
    "A": (".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"),
    "C": (".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."),
    "I": ("#####", "..#..", "..#..", "..#..", "..#..", "..#..", "#####"),
    "L": ("#....", "#....", "#....", "#....", "#....", "#....", "#####"),
    "R": ("####.", "#...#", "#...#", "####.", "#..#.", "#...#", "#...#"),
    "S": (".####", "#....", "#....", ".###.", "....#", "....#", "####."),
    "T": ("#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."),
    "U": ("#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."),
}

# The shipped labels. The words are generic beverage descriptors, not brands.
LABELS = (("can_classic_label.png", "CLASSIC"), ("can_citrus_label.png", "CITRUS"))


def word_width(word: str) -> int:
    """Return the rendered width in pixels of one whole word."""
    return (len(word) * (GLYPH_WIDTH + GLYPH_GAP) - GLYPH_GAP) * GLYPH_SCALE


def draw_word(pixels: list[list[tuple[int, int, int]]], word: str, left: int, top: int) -> None:
    """Stamp one word into the pixel grid, wrapping horizontally at the seam."""
    cursor = left
    for character in word:
        glyph = FONT[character]
        for row_index, row in enumerate(glyph):
            for column_index, cell in enumerate(row):
                if cell != "#":
                    continue
                for dy in range(GLYPH_SCALE):
                    y = top + row_index * GLYPH_SCALE + dy
                    if not 0 <= y < HEIGHT:
                        continue
                    for dx in range(GLYPH_SCALE):
                        x = (cursor + column_index * GLYPH_SCALE + dx) % WIDTH
                        pixels[y][x] = INK
        cursor += (GLYPH_WIDTH + GLYPH_GAP) * GLYPH_SCALE


def render(word: str) -> bytes:
    """Render one label as a raw RGB pixel grid serialised into a PNG byte string."""
    pixels = [[FIELD for _ in range(WIDTH)] for _ in range(HEIGHT)]
    for first, last in RULE_ROWS:
        for y in range(first, last):
            pixels[y] = [RULE for _ in range(WIDTH)]
    for y in range(BAND_TOP, BAND_BOTTOM):
        pixels[y] = [BAND for _ in range(WIDTH)]

    span = WIDTH // REPEATS
    text_width = word_width(word)
    text_top = BAND_TOP + (BAND_BOTTOM - BAND_TOP - GLYPH_HEIGHT * GLYPH_SCALE) // 2
    for repeat in range(REPEATS):
        draw_word(pixels, word, repeat * span + (span - text_width) // 2, text_top)

    raw = bytearray()
    for row in pixels:
        raw.append(0)
        for red, green, blue in row:
            raw.extend((red, green, blue))
    return encode_png(bytes(raw))


def encode_png(raw: bytes) -> bytes:
    """Wrap filtered RGB scanlines in the smallest conforming PNG container."""

    def chunk(kind: bytes, payload: bytes) -> bytes:
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))

    header = struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def main() -> int:
    """Rewrite every shipped label and report the ones whose bytes changed."""
    changed = []
    for name, word in LABELS:
        path = TEXTURE_DIR / name
        content = render(word)
        if not path.is_file() or path.read_bytes() != content:
            path.write_bytes(content)
            changed.append(name)
    for name in changed:
        print(f"rewrote {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
