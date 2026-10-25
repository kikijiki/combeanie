# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Install a label texture for Gazebo: the one place a cylinder's UV half-turn is applied."""
# Every shipped label strip is stored upright (readable) on disk. Gazebo unwraps a cylinder
# primitive's lateral surface rotated by half a turn, so a strip applied to a cylinder shows
# upside down with its back panel first; a mesh with its own UVs (the contour bottle) shows it
# upright. The manifest (config/generated_product_labels.yaml, written by tools/label_factory.py)
# names the half-turn per texture as ``texture_rotation_deg``, and the launch file installs the
# strip through this module. A texture the manifest does not list (the hand-made can labels) is
# installed unchanged.
#
# Standard library only: the launch file runs where Pillow and numpy are not guaranteed.

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import struct
import tempfile
import zlib

import yaml

_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_CHANNELS = {0: 1, 2: 3, 4: 2, 6: 4}
SUPPORTED_ROTATIONS_DEG = (0, 180)


def load_texture_rotations(manifest_path: str | Path) -> dict[str, int]:
    """Map texture filename to the rotation (degrees) to install it with."""
    manifest = yaml.safe_load(Path(manifest_path).read_text(encoding="utf-8"))
    rotations: dict[str, int] = {}
    for product in manifest.get("products", []):
        rotation = int(product.get("texture_rotation_deg", 0))
        if rotation not in SUPPORTED_ROTATIONS_DEG:
            raise RuntimeError(
                f"{product['texture']}: texture_rotation_deg must be one of "
                f"{SUPPORTED_ROTATIONS_DEG}, got {rotation}"
            )
        rotations[product["texture"]] = rotation
    return rotations


def _chunks(data: bytes):
    if not data.startswith(_SIGNATURE):
        raise ValueError("not a PNG")
    pos = len(_SIGNATURE)
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        yield data[pos + 4 : pos + 8], data[pos + 8 : pos + 8 + length]
        pos += 12 + length


def _paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def _unfilter(raw: bytes, height: int, stride: int, bpp: int) -> list[bytearray]:
    rows: list[bytearray] = []
    previous = bytearray(stride)
    pos = 0
    for _ in range(height):
        kind = raw[pos]
        line = bytearray(raw[pos + 1 : pos + 1 + stride])
        pos += 1 + stride
        if kind == 1:
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 255
        elif kind == 2:
            for i in range(stride):
                line[i] = (line[i] + previous[i]) & 255
        elif kind == 3:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((left + previous[i]) >> 1)) & 255
        elif kind == 4:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                upleft = previous[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + _paeth(left, previous[i], upleft)) & 255
        elif kind != 0:
            raise ValueError(f"unknown PNG filter {kind}")
        rows.append(line)
        previous = line
    return rows


def _chunk(tag: bytes, payload: bytes) -> bytes:
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", zlib.crc32(tag + payload))
    )


def rotate_png_180(data: bytes) -> bytes:
    """The same pixels turned half a turn (rows reversed, pixels reversed within each row)."""
    header = None
    palette = b""
    idat = []
    for tag, payload in _chunks(data):
        if tag == b"IHDR":
            header = payload
        elif tag == b"PLTE":
            palette = payload
        elif tag == b"IDAT":
            idat.append(payload)
    if header is None:
        raise ValueError("PNG has no IHDR")
    width, height, depth, colour, _, _, interlace = struct.unpack(">IIBBBBB", header)
    if depth != 8 or colour not in _CHANNELS or interlace != 0 or palette:
        raise ValueError("only non-interlaced 8-bit grey/RGB(A) PNGs can be rotated")
    bpp = _CHANNELS[colour]
    stride = width * bpp
    rows = _unfilter(zlib.decompress(b"".join(idat)), height, stride, bpp)
    out = bytearray()
    for line in reversed(rows):
        out.append(0)
        for x in range(width - 1, -1, -1):
            out += line[x * bpp : (x + 1) * bpp]
    return (
        _SIGNATURE
        + _chunk(b"IHDR", header)
        + _chunk(b"IDAT", zlib.compress(bytes(out), 9))
        + _chunk(b"IEND", b"")
    )


def default_cache_dir() -> Path:
    """Where rotated copies live: per user and across shells, since the copies are content-named.

    A shell's own temporary directory would redo the rotation, several seconds of pure-Python PNG
    work, at the start of every launch.
    """
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    persistent = Path(base) / "restocker" / "label_cache"
    try:
        persistent.mkdir(parents=True, exist_ok=True)
    except OSError:  # a sandbox with no writable home
        return Path(tempfile.gettempdir()) / "restocker_label_cache"
    return persistent


def install_texture(source: Path, rotation_deg: int, cache_dir: str | Path | None = None) -> Path:
    """Path Gazebo should load for ``source``: itself, or a content-named rotated copy."""
    if rotation_deg not in SUPPORTED_ROTATIONS_DEG:
        raise RuntimeError(f"unsupported texture rotation {rotation_deg}")
    if rotation_deg == 0:
        return source
    data = source.read_bytes()
    cache = Path(cache_dir) if cache_dir else default_cache_dir()
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / f"{hashlib.sha256(data).hexdigest()[:16]}_r{rotation_deg}_{source.name}"
    if not target.is_file():
        scratch = target.with_name(f".{os.getpid()}.{target.name}")
        scratch.write_bytes(rotate_png_180(data))
        os.replace(scratch, target)
    return target
