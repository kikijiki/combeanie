# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Offline CPU renderer: unwrapped label texture (and optional bottle mesh) -> eight views.

Boxes (CMB-119) have their own path, ``render_box_views``: the texture is a face atlas and each
view is a ray-cast of the oriented box with every face sampled from its own atlas rectangle."""
# Same approach as the CMB-101 identity experiment: ray-cast a vertical body, sample the unwrapped
# texture bilinearly (u = once around the barrel, v = top to bottom), Lambert plus a little
# specular, supersample to tame minification. numpy only. The texture is the source of truth;
# the views are a pure function of (texture bytes, radius, height, optional profile, parameters),
# so rendering twice gives byte-identical PNGs and a registry can be rebuilt from the textures.
#
# Differences from the experiment, all so the output is a registration exemplar and not a test
# query: the camera frame is fitted to the product (a tight crop at a fixed scale, as a
# registration turntable photo would be), the background is flat grey, lighting is the nominal
# one, and nothing is randomised.
#
# The texture is expected upright (readable as stored): u runs around the barrel and v top to
# bottom here, with no rotation applied. The half-turn Gazebo's cylinder primitive needs is
# applied by restocker_gazebo.label_textures when the texture is installed for the simulator.
#
# A bottle mesh enters only as a surface of revolution (radius against height read from the OBJ's
# vertices, axis z, centred), the same shape the generator emitted, so the silhouette and the
# shading follow the waist and ribs while the label still wraps by the mesh's own u = angle,
# v = height UVs. Without a mesh the body is the catalogue cylinder.

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from restocker_perception.registry.imageio import read_image, resize_area

VIEW_COUNT = 8
PIXELS_PER_METRE = 3000.0  # about 0.33 mm per pixel on the axis, the CMB-101 inspection scale
STANDOFF_M = 0.30
SUPERSAMPLE = 2
MARGIN = 0.04
BACKGROUND = 0.5
# Direction from the surface towards the light: above-right of the camera, which is at -z.
LIGHT_DIR = (0.3, 0.3, -1.0)
AMBIENT = 0.55
SPECULAR = 0.10
MARCH_STEPS = 96
BISECT_STEPS = 10


@dataclass(frozen=True)
class Profile:
    """Surface of revolution: radius (m) at heights (m, centred on 0), ascending."""

    heights: np.ndarray
    radii: np.ndarray

    def radius_at(self, y: np.ndarray) -> np.ndarray:
        inside = (y >= self.heights[0]) & (y <= self.heights[-1])
        return np.where(inside, np.interp(y, self.heights, self.radii), -1.0)

    def slope_at(self, y: np.ndarray) -> np.ndarray:
        step = 0.5e-3
        return (self.radius_at(y + step) - self.radius_at(y - step)) / (2 * step)

    @property
    def max_radius(self) -> float:
        return float(self.radii.max())


def profile_from_obj(path: str | Path) -> Profile:
    """Read the radius-against-height profile from a z-axis OBJ (largest radius at each z)."""
    by_height: dict[float, float] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            x, y, z = (float(token) for token in line.split()[1:4])
            key = round(z, 6)
            by_height[key] = max(by_height.get(key, 0.0), math.hypot(x, y))
    if len(by_height) < 2:
        raise ValueError(f"{path} has no usable vertex rows")
    heights = np.array(sorted(by_height), dtype=np.float64)
    radii = np.array([by_height[h] for h in sorted(by_height)], dtype=np.float64)
    return Profile(heights, radii)


def load_texture(path: str | Path, max_width: int = 2048) -> np.ndarray:
    """Texture as float32 HxWx3 in [0, 1], area-downsized to at most ``max_width`` wide."""
    image = read_image(path)
    if image.shape[1] > max_width:
        height = round(image.shape[0] * max_width / image.shape[1])
        image = resize_area(image, max_width, height)
    return image.astype(np.float32) / 255.0


def _sample(texture: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Bilinear sample, u wrapping around, v clamped."""
    height, width, _ = texture.shape
    x = (u % 1.0) * width - 0.5
    y = np.clip(v, 0.0, 1.0) * (height - 1)
    x0 = np.floor(x).astype(int)
    fx = (x - x0)[..., None]
    y0 = np.floor(y).astype(int)
    fy = (y - y0)[..., None]
    y1 = np.minimum(y0 + 1, height - 1)
    x1 = (x0 + 1) % width
    x0 = x0 % width
    return (
        texture[y0, x0] * (1 - fx) * (1 - fy)
        + texture[y0, x1] * fx * (1 - fy)
        + texture[y1, x0] * (1 - fx) * fy
        + texture[y1, x1] * fx * fy
    )


def _hit_cylinder(origin_z: float, d: np.ndarray, radius: float, height: float):
    a = d[..., 0] ** 2 + d[..., 2] ** 2
    b = 2 * origin_z * d[..., 2]
    c = origin_z**2 - radius**2
    disc = b * b - 4 * a * c
    ok = disc > 0
    t = (-b - np.sqrt(np.where(ok, disc, 0.0))) / (2 * a)
    y = t * d[..., 1]
    mask = ok & (t > 0) & (np.abs(y) <= height / 2)
    return mask, t, None


def _hit_profile(origin_z: float, d: np.ndarray, profile: Profile):
    """March each ray through the bounding cylinder, then bisect the first inside sample."""
    rmax = profile.max_radius
    a = d[..., 0] ** 2 + d[..., 2] ** 2
    b = 2 * origin_z * d[..., 2]
    c = origin_z**2 - rmax**2
    disc = b * b - 4 * a * c
    candidate = disc > 0
    root = np.sqrt(np.where(candidate, disc, 0.0))
    t_near = np.where(candidate, (-b - root) / (2 * a), 0.0)
    t_far = np.where(candidate, (-b + root) / (2 * a), 0.0)

    def inside(t: np.ndarray) -> np.ndarray:
        x, y, z = t * d[..., 0], t * d[..., 1], origin_z + t * d[..., 2]
        return np.hypot(x, z) - profile.radius_at(y) <= 0.0

    low = t_near.copy()
    high = t_near.copy()
    found = np.zeros(d.shape[:2], dtype=bool)
    previous = t_near.copy()
    for step in range(MARCH_STEPS + 1):
        t = t_near + (t_far - t_near) * step / MARCH_STEPS
        now = inside(t) & candidate & ~found
        low = np.where(now, previous, low)
        high = np.where(now, t, high)
        found |= now
        previous = t
    for _ in range(BISECT_STEPS):
        mid = (low + high) / 2
        is_in = inside(mid)
        high = np.where(found & is_in, mid, high)
        low = np.where(found & ~is_in, mid, low)
    return found, high, profile


def render_view(
    texture: np.ndarray,
    radius: float,
    height: float,
    yaw_deg: float,
    *,
    profile: Profile | None = None,
) -> np.ndarray:
    """One exemplar view: HxWx3 uint8 crop of the body turned by ``yaw_deg`` about its axis."""
    extent = profile.max_radius if profile is not None else radius
    out_w = int(math.ceil(2 * extent * (1 + MARGIN) * PIXELS_PER_METRE))
    out_h = int(math.ceil(height * (1 + MARGIN) * PIXELS_PER_METRE))
    scale = SUPERSAMPLE
    width, rows = out_w * scale, out_h * scale
    focal = PIXELS_PER_METRE * STANDOFF_M * scale
    origin_z = -STANDOFF_M
    xs = np.arange(width) - width / 2 + 0.5
    ys = np.arange(rows) - rows / 2 + 0.5
    uu, vv = np.meshgrid(xs, ys)
    d = np.stack([uu / focal, -vv / focal, np.ones_like(uu)], axis=-1)
    d /= np.linalg.norm(d, axis=-1, keepdims=True)

    if profile is None:
        mask, t, _ = _hit_cylinder(origin_z, d, radius, height)
    else:
        mask, t, _ = _hit_profile(origin_z, d, profile)
    hit_x, hit_y, hit_z = t * d[..., 0], t * d[..., 1], origin_z + t * d[..., 2]
    if profile is not None:
        span = float(profile.heights[-1] - profile.heights[0])
        centre = float(profile.heights[-1] + profile.heights[0]) / 2
    else:
        span, centre = height, 0.0
    angle = np.arctan2(hit_x, -hit_z)  # 0 faces the camera at yaw 0
    u = angle / (2 * math.pi) + yaw_deg / 360.0
    v = 0.5 - (hit_y - centre) / span
    colour = _sample(texture, u, v)

    rho = np.maximum(np.hypot(hit_x, hit_z), 1e-9)
    slope = profile.slope_at(hit_y) if profile is not None else np.zeros_like(hit_y)
    normal = np.stack([hit_x / rho, -slope, hit_z / rho], axis=-1)
    normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
    light = np.array(LIGHT_DIR, dtype=np.float64)
    light /= np.linalg.norm(light)
    lambert = np.clip((normal * light).sum(-1), 0.0, 1.0)
    half_vector = light - d
    half_vector /= np.linalg.norm(half_vector, axis=-1, keepdims=True) + 1e-9
    specular = np.clip((normal * half_vector).sum(-1), 0.0, 1.0) ** 24
    shade = (AMBIENT + (1 - AMBIENT) * lambert)[..., None]
    lit = np.clip(
        colour * shade / (AMBIENT + (1 - AMBIENT) * 0.9) + SPECULAR * specular[..., None],
        0.0,
        1.0,
    )
    frame = np.where(mask[..., None], lit, BACKGROUND)
    frame = frame.reshape(out_h, scale, out_w, scale, 3).mean(axis=(1, 3))
    return (np.clip(frame, 0.0, 1.0) * 255 + 0.5).astype(np.uint8)


def render_views(
    texture: np.ndarray,
    radius: float,
    height: float,
    *,
    profile: Profile | None = None,
    count: int = VIEW_COUNT,
) -> list[np.ndarray]:
    """The registry's views: ``count`` renders at 360/count degree steps starting at 0."""
    return [
        render_view(texture, radius, height, k * 360.0 / count, profile=profile)
        for k in range(count)
    ]


# --------------------------------------------------------------------------------------------
# Boxes
# --------------------------------------------------------------------------------------------

BOX_ELEVATION_DEG = 20.0  # registry views look slightly down, so the top face shows a little
BOX_MARGIN = 0.08
BOX_ATLAS_MAX_WIDTH = 4096


@dataclass(frozen=True)
class BoxFace:
    """One face of the atlas: its pixel rectangle and its frame in the product frame."""

    rect_px: tuple[int, int, int, int]  # x, y, w, h; origin top left
    normal: np.ndarray
    image_right: np.ndarray
    image_up: np.ndarray


@dataclass(frozen=True)
class BoxLayout:
    """A box's size (x width, y depth, z height; the front face looks along -y) and atlas."""

    size_xyz_m: tuple[float, float, float]
    atlas_px: tuple[int, int]
    faces: dict[str, BoxFace]

    @staticmethod
    def from_manifest_row(row: dict) -> BoxLayout:
        faces = {
            name: BoxFace(
                tuple(int(v) for v in face["rect_px"]),
                np.array(face["normal"], dtype=np.float64),
                np.array(face["image_right"], dtype=np.float64),
                np.array(face["image_up"], dtype=np.float64),
            )
            for name, face in row["faces"].items()
        }
        return BoxLayout(
            tuple(float(v) for v in row["size_xyz_m"]),
            tuple(int(v) for v in row["atlas_px"]),
            faces,
        )

    @property
    def bounding_radius_m(self) -> float:
        return 0.5 * math.hypot(self.size_xyz_m[0], self.size_xyz_m[1])


def box_camera_direction(yaw_deg: float, elevation_deg: float) -> np.ndarray:
    """Unit vector from the box centre to the camera, in the product frame.

    ``yaw_deg`` turns the box counter-clockwise seen from above relative to a camera that, at
    yaw 0 and elevation 0, looks at the front face (the camera stands on the -y side).
    """
    yaw, elevation = math.radians(yaw_deg), math.radians(elevation_deg)
    return np.array(
        [
            -math.sin(yaw) * math.cos(elevation),
            -math.cos(yaw) * math.cos(elevation),
            math.sin(elevation),
        ]
    )


def _sample_clamped(texture: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Bilinear sample at pixel coordinates (pixel centres at +0.5), clamped to the image."""
    height, width, _ = texture.shape
    x = np.clip(x - 0.5, 0.0, width - 1.0)
    y = np.clip(y - 0.5, 0.0, height - 1.0)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0 + 1, width - 1), np.minimum(y0 + 1, height - 1)
    fx, fy = (x - x0)[..., None], (y - y0)[..., None]
    return (
        texture[y0, x0] * (1 - fx) * (1 - fy)
        + texture[y0, x1] * fx * (1 - fy)
        + texture[y1, x0] * (1 - fx) * fy
        + texture[y1, x1] * fx * fy
    )


def render_box_view(
    texture: np.ndarray,
    layout: BoxLayout,
    yaw_deg: float,
    elevation_deg: float = BOX_ELEVATION_DEG,
) -> np.ndarray:
    """One exemplar view of a box: HxWx3 uint8, every face upright and unmirrored."""
    size = np.array(layout.size_xyz_m)
    half = size / 2.0
    towards = box_camera_direction(yaw_deg, elevation_deg)
    forward = -towards
    up_world = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up_world)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    diagonal = math.hypot(size[0], size[1])
    elevation = math.radians(elevation_deg)
    out_w = int(math.ceil(diagonal * (1 + BOX_MARGIN) * PIXELS_PER_METRE))
    out_h = int(
        math.ceil(
            (size[2] * math.cos(elevation) + diagonal * math.sin(elevation))
            * (1 + BOX_MARGIN)
            * PIXELS_PER_METRE
        )
    )
    scale = SUPERSAMPLE
    width, rows = out_w * scale, out_h * scale
    focal = PIXELS_PER_METRE * STANDOFF_M * scale
    xs = np.arange(width) - width / 2 + 0.5
    ys = np.arange(rows) - rows / 2 + 0.5
    uu, vv = np.meshgrid(xs, ys)
    cam = np.stack([uu / focal, -vv / focal, np.ones_like(uu)], axis=-1)
    cam /= np.linalg.norm(cam, axis=-1, keepdims=True)
    direction = cam[..., 0:1] * right + cam[..., 1:2] * up + cam[..., 2:3] * forward
    origin = towards * STANDOFF_M
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-half - origin) / direction[..., :]
        t2 = (half - origin) / direction[..., :]
    t_near = np.minimum(t1, t2)
    t_far = np.maximum(t1, t2)
    enter = t_near.max(axis=-1)
    leave = t_far.min(axis=-1)
    mask = (enter <= leave) & (leave > 0) & (enter > 0)
    axis = t_near.argmax(axis=-1)
    point = origin + enter[..., None] * direction
    sign = np.where(
        np.take_along_axis(direction, axis[..., None], axis=-1)[..., 0] > 0, -1.0, 1.0
    )  # entering through the face the ray travels towards: normal opposes the ray
    colour = np.full((rows, width, 3), BACKGROUND, dtype=np.float32)
    normal_world = np.zeros((rows, width, 3), dtype=np.float64)
    for face in layout.faces.values():
        axis_index = int(np.argmax(np.abs(face.normal)))
        facing = mask & (axis == axis_index) & (sign == face.normal[axis_index])
        if not facing.any():
            continue
        span_r = float(np.abs(face.image_right) @ size)
        span_u = float(np.abs(face.image_up) @ size)
        a = point[facing] @ face.image_right / span_r + 0.5
        b = point[facing] @ face.image_up / span_u + 0.5
        x0, y0, w, h = face.rect_px
        atlas_w, atlas_h = layout.atlas_px
        scale_x = texture.shape[1] / atlas_w
        scale_y = texture.shape[0] / atlas_h
        px = (x0 + a * w) * scale_x
        py = (y0 + (1.0 - b) * h) * scale_y
        colour[facing] = _sample_clamped(texture, px, py)
        normal_world[facing] = face.normal
    n_cam = np.stack([normal_world @ right, normal_world @ up, normal_world @ forward], axis=-1)
    light = np.array(LIGHT_DIR, dtype=np.float64)
    light /= np.linalg.norm(light)
    lambert = np.clip((n_cam * light).sum(-1), 0.0, 1.0)
    shade = (AMBIENT + (1 - AMBIENT) * lambert)[..., None]
    lit = np.clip(colour * shade / (AMBIENT + (1 - AMBIENT) * 0.9), 0.0, 1.0)
    frame = np.where(mask[..., None], lit, BACKGROUND)
    frame = frame.reshape(out_h, scale, out_w, scale, 3).mean(axis=(1, 3))
    return (np.clip(frame, 0.0, 1.0) * 255 + 0.5).astype(np.uint8)


def render_box_views(
    texture: np.ndarray,
    layout: BoxLayout,
    *,
    elevation_deg: float = BOX_ELEVATION_DEG,
    count: int = VIEW_COUNT,
) -> list[np.ndarray]:
    """The registry's views of a box: ``count`` yaws at 360/count degrees (0 = front face on).

    With the default eight, yaws 0, 90, 180 and 270 face the front, left, back and right sides
    squarely (the box turns counter-clockwise from above) and the odd multiples of 45 show two
    faces at once.
    """
    return [
        render_box_view(texture, layout, k * 360.0 / count, elevation_deg) for k in range(count)
    ]
