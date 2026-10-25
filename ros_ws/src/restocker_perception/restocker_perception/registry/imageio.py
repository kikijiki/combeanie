# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Image input and output for the registry: deterministic PNG writer, PNG reader, area resize."""
# Views are stored as PNG and hashed, so the writer must produce the same bytes on every machine:
# it is our own (fixed filter, fixed zlib level), never Pillow's. Reading uses Pillow when it is
# installed (any format) and falls back to a small PNG decoder, so the package and its tests run
# without an imaging library.

from __future__ import annotations

from pathlib import Path
import struct
import zlib

import numpy as np

_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _chunk(kind: bytes, payload: bytes) -> bytes:
    body = kind + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))


def encode_png(pixels: np.ndarray) -> bytes:
    """Encode an HxWx3 uint8 array as a deterministic PNG."""
    array = np.asarray(pixels)
    if array.ndim != 3 or array.shape[2] != 3 or array.dtype != np.uint8:
        raise ValueError("encode_png takes an HxWx3 uint8 array")
    height, width = array.shape[:2]
    rows = array.reshape(height, -1)
    stream = b"".join(b"\x00" + rows[r].tobytes() for r in range(height))
    return (
        _SIGNATURE
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(stream, 6))
        + _chunk(b"IEND", b"")
    )


def decode_png(data: bytes) -> np.ndarray:
    """Decode an 8-bit, non-interlaced PNG (gray, RGB, gray+alpha or RGBA) to HxWx3 uint8."""
    if data[:8] != _SIGNATURE:
        raise ValueError("not a PNG")
    offset, idat, header = 8, b"", None
    while offset < len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        kind = data[offset + 4 : offset + 8]
        payload = data[offset + 8 : offset + 8 + length]
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", payload)
        elif kind == b"IDAT":
            idat += payload
        offset += 12 + length
    if header is None:
        raise ValueError("PNG has no header")
    width, height, depth, colour, _compression, _filter, interlace = header
    channels = {0: 1, 2: 3, 4: 2, 6: 4}.get(colour)
    if depth != 8 or channels is None or interlace != 0:
        raise ValueError("only 8-bit non-interlaced gray/RGB/RGBA PNG is supported without Pillow")
    raw = zlib.decompress(idat)
    stride = width * channels
    out = np.zeros((height, stride), dtype=np.uint8)
    previous = bytearray(stride)
    for row in range(height):
        start = row * (stride + 1)
        mode = raw[start]
        line = bytearray(raw[start + 1 : start + 1 + stride])
        if mode == 1:
            pixels = np.frombuffer(bytes(line), dtype=np.uint8).reshape(width, channels)
            line = bytearray(np.cumsum(pixels, axis=0, dtype=np.uint8).tobytes())
        elif mode == 2:
            line = bytearray(
                (np.frombuffer(bytes(line), np.uint8) + np.frombuffer(bytes(previous), np.uint8))
                .astype(np.uint8)
                .tobytes()
            )
        elif mode == 3:
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                line[i] = (line[i] + ((left + previous[i]) >> 1)) & 0xFF
        elif mode == 4:
            for i in range(stride):
                if i >= channels:
                    a, c = line[i - channels], previous[i - channels]
                else:
                    a = c = 0
                b = previous[i]
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                predictor = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
                line[i] = (line[i] + predictor) & 0xFF
        elif mode != 0:
            raise ValueError(f"bad PNG filter {mode}")
        out[row] = np.frombuffer(bytes(line), dtype=np.uint8)
        previous = line
    image = out.reshape(height, width, channels)
    if channels in (1, 2):
        return np.repeat(image[:, :, :1], 3, axis=2)
    return image[:, :, :3].copy()


def read_image(path: str | Path) -> np.ndarray:
    """Read an image file as HxWx3 uint8 RGB."""
    path = Path(path)
    try:
        from PIL import Image  # noqa: PLC0415 - optional, only for non-PNG input
    except ImportError:
        return decode_png(path.read_bytes())
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.uint8)


def write_png(path: str | Path, pixels: np.ndarray) -> str:
    """Write a deterministic PNG and return its SHA-256."""
    import hashlib  # noqa: PLC0415

    data = encode_png(pixels)
    Path(path).write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def _area_weights(source: int, target: int) -> np.ndarray:
    """Matrix (target x source) of box-filter overlap weights, rows sum to one."""
    edges = np.linspace(0.0, source, target + 1)
    weights = np.zeros((target, source), dtype=np.float32)
    for row in range(target):
        low, high = edges[row], edges[row + 1]
        first, last = int(np.floor(low)), int(np.ceil(high))
        for column in range(first, min(last, source)):
            weights[row, column] = min(high, column + 1) - max(low, column)
    return weights / weights.sum(axis=1, keepdims=True)


def resize_area(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Area-averaging resize of a float or uint8 HxWxC image (uint8 in, uint8 out)."""
    source = image.astype(np.float32)
    wy = _area_weights(source.shape[0], height)
    wx = _area_weights(source.shape[1], width)
    resized = np.einsum("ys,sxc->yxc", wy, source)
    resized = np.einsum("xs,ysc->yxc", wx, resized)
    if image.dtype == np.uint8:
        return np.clip(resized + 0.5, 0, 255).astype(np.uint8)
    return resized
