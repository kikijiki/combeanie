#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Generate a fictional beverage catalogue: fully unwrapped label textures and their manifest.

Every brand, flavour, logo, ingredient list and barcode is invented. Nothing here depicts a real
product. A texture is the whole lateral surface of a product cylinder: u runs once around the
circumference, v along the axis, so the strip carries a front panel (name, variant, logo, volume),
a narrow side panel, and a back panel (ingredients, nutrition table, disclaimers, recycling marks
and an EAN-13 barcode whose check digit is valid and whose bars decode).

Look-alike pairs share a layout and palette family and differ in the flavour word and one accent,
so a colour histogram cannot tell them apart.

Boxes (cartons, a slim box; CMB-119) are not bodies of revolution. Their texture is an atlas, the
unfolded net of the six faces at one pixel density: front, right, back and left side by side in
one row (the order a viewer walking round the box sees them), top and bottom below. The manifest
row carries a ``faces`` table (pixel rectangle, outward normal, and the product-frame directions
the image's right and up edges point at) so the Gazebo mesh, the registry renderer and perception
all read one layout. Every face is stored upright and unmirrored as seen from outside.

Output is a function of the seeds in CATALOGUE below: rerunning rewrites byte-identical files. The
textures go to restocker_gazebo/materials/textures and the manifest to
restocker_gazebo/config/generated_product_labels.yaml. Needs Pillow (in the dev shell).

    python3 tools/label_factory.py            # rewrite everything
    python3 tools/label_factory.py --check    # fail if a file would change
"""

from __future__ import annotations

import argparse
import colorsys
from dataclasses import dataclass
from pathlib import Path
import random
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
GAZEBO = ROOT / "ros_ws" / "src" / "restocker_gazebo"
TEXTURE_DIR = GAZEBO / "materials" / "textures"
MANIFEST = GAZEBO / "config" / "generated_product_labels.yaml"
CATALOG = (
    ROOT / "ros_ws" / "src" / "restocker_description" / "config" / "product_collision_catalog.yaml"
)
CATALOG_BEGIN = (
    "# BEGIN generated products (tools/label_factory.py; do not edit between the markers)"
)
CATALOG_END = "# END generated products"

TEXTURE_WIDTH = 1536
SUPERSAMPLE = 2

# Catalogue geometry (metres) the label has to wrap. Keep in step with
# restocker_description/config/product_collision_catalog.yaml; test_label_factory pins it.
SHAPES = {
    "can": {"radius": 0.033, "height": 0.122, "volume": "330 mL", "band": (0.0, 1.0)},
    "small_bottle": {"radius": 0.034, "height": 0.200, "volume": "500 mL", "band": (0.24, 0.80)},
    # Contoured PET bottle: waisted with grip ribs (tools/generate_bottle_meshes.py). Radius and
    # height are the collision cylinder's, i.e. the mesh's maximum radius and full height.
    "contour_bottle": {
        "radius": 0.036,
        "height": 0.230,
        "volume": "750 mL",
        "band": (0.20, 0.72),
        "visual_mesh": "contour_bottle.obj",
    },
    "large_bottle": {"radius": 0.045, "height": 0.290, "volume": "1.5 L", "band": (0.22, 0.82)},
}

# Box variants. Product frame: x = width (to the right as seen from the front), y = depth (the
# front face looks along -y), z = height; the origin is the box centre. Sizes are metres.
BOX_PIXELS_PER_METRE = 8000.0
BOX_VARIANTS = {
    # 200 mL carton, 1 L carton and a slim box (the barcode-less one).
    "carton_200": {"size": (0.065, 0.040, 0.105), "volume": "200 mL", "mass_kg": 0.215},
    "carton_1000": {"size": (0.095, 0.060, 0.165), "volume": "1 L", "mass_kg": 1.030},
    "slim_box": {"size": (0.055, 0.025, 0.150), "volume": "330 mL", "mass_kg": 0.345},
}
BOX_FACES = ("front", "right", "back", "left", "top", "bottom")
# Outward normal and the directions of the image's right and up edges, in the product frame. All
# six are right-handed (image_right x image_up = normal), so a face is never mirrored.
BOX_FACE_FRAMES = {
    "front": ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
    "right": ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
    "back": ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),
    "left": ((-1, 0, 0), (0, -1, 0), (0, 0, 1)),
    "top": ((0, 0, 1), (1, 0, 0), (0, 1, 0)),
    "bottom": ((0, 0, -1), (1, 0, 0), (0, -1, 0)),
}


def box_layout(size: tuple[float, float, float]) -> dict:
    """Atlas size and the pixel rectangle (x, y, w, h; origin top left) of each face."""
    sx, sy, sz = size
    ppm = BOX_PIXELS_PER_METRE
    wx, wy, hz = round(sx * ppm), round(sy * ppm), round(sz * ppm)
    rects = {
        "front": (0, 0, wx, hz),
        "right": (wx, 0, wy, hz),
        "back": (wx + wy, 0, wx, hz),
        "left": (2 * wx + wy, 0, wy, hz),
        "top": (0, hz, wx, wy),
        "bottom": (wx, hz, wx, wy),
    }
    return {"atlas": (2 * wx + 2 * wy, hz + wy), "rects": rects}


# ---- invented vocabulary ---------------------------------------------------------------------

BRAND_START = (
    "Zor",
    "Mi",
    "Vel",
    "Kora",
    "Tala",
    "Nuo",
    "Bri",
    "Fenn",
    "Sora",
    "Qui",
    "Dal",
    "Pyr",
)
BRAND_END = (
    "vana",
    "zuki",
    "lume",
    "tara",
    "bex",
    "nova",
    "ria",
    "mona",
    "pix",
    "lo",
    "dex",
    "ya",
)
FLAVOURS = (
    "Citrus",
    "Cherry",
    "Lime",
    "Mango",
    "Peach",
    "Berry",
    "Ginger",
    "Melon",
    "Apple",
    "Mint",
    "Grape",
    "Pineapple",
    "Lemon",
    "Guava",
    "Plum",
    "Coconut",
)
KINDS = ("Sparkling", "Soda", "Iced Tea", "Fizz", "Tonic", "Cooler")
INGREDIENTS = (
    "Carbonated water",
    "Cane sugar",
    "Citric acid",
    "Natural flavours",
    "Sodium citrate",
    "Ascorbic acid",
    "Caramel colour",
    "Potassium sorbate",
    "Stevia leaf extract",
    "Apple juice concentrate",
    "Malic acid",
    "Gum arabic",
    "Vegetable extract (colour)",
)
BOX_KINDS = ("Juice Drink", "Nectar", "Fruit Blend")
SLIM_KINDS = ("Oat Drink", "Almond Drink", "Soy Drink")
BOX_INGREDIENTS = (
    "Water",
    "Fruit juice from concentrate",
    "Cane sugar",
    "Citric acid",
    "Natural flavours",
    "Ascorbic acid",
    "Pectin",
    "Vegetable extract (colour)",
    "Sea salt",
    "Calcium carbonate",
)
DISCLAIMERS = (
    "Best before: see base of container.",
    "Contains no fruit juice. Shake gently. Open slowly.",
    "Store in a cool dry place away from sunlight.",
    "Please recycle. Deposit applies where available.",
    "Not suitable for children under 4 years.",
    "Product of Nowhere. Bottled for Examplia Beverages Ltd.",
)


@dataclass(frozen=True)
class Spec:
    """One catalogue entry before rendering."""

    sku: str
    shape: str
    geometry_key: str
    seed: int
    brand: str
    flavour: str
    kind: str
    hue: float
    accent_hue: float
    layout_seed: int
    pair: str
    # Boxes only: the BOX_VARIANTS key, and whether the back panel carries an EAN-13.
    variant: str = ""
    barcode: bool = True

    @property
    def is_box(self) -> bool:
        return self.shape == "box"

    @property
    def texture(self) -> str:
        return f"{self.sku.lower()}.png"

    @property
    def texture_rotation_deg(self) -> int:
        # Strips are stored upright. Measured in Gazebo (Card 110): a cylinder primitive maps the
        # strip rotated by half a turn (upside down, back panel first); the contour mesh's own
        # UVs (generate_bottle_meshes.py) show it upright and unmirrored. restocker_gazebo's
        # label_textures.install_texture applies this turn when it installs the texture.
        # A box atlas has its own per-face UVs (generate_box_meshes.py), like the contour mesh.
        return 0 if self.shape in ("contour_bottle", "box") else 180


# ---- EAN-13 ----------------------------------------------------------------------------------

_L = (
    "0001101",
    "0011001",
    "0010011",
    "0111101",
    "0100011",
    "0110001",
    "0101111",
    "0111011",
    "0110111",
    "0001011",
)
_G = (
    "0100111",
    "0110011",
    "0011011",
    "0100001",
    "0011101",
    "0111001",
    "0000101",
    "0010001",
    "0001001",
    "0010111",
)
_R = (
    "1110010",
    "1100110",
    "1101100",
    "1000010",
    "1011100",
    "1001110",
    "1010000",
    "1000100",
    "1001000",
    "1110100",
)
_PARITY = (
    "LLLLLL",
    "LLGLGG",
    "LLGGLG",
    "LLGGGL",
    "LGLLGG",
    "LGGLLG",
    "LGGGLL",
    "LGLGLG",
    "LGLGGL",
    "LGGLGL",
)


def ean13_check_digit(first12: str) -> int:
    total = sum(int(d) * (3 if i % 2 else 1) for i, d in enumerate(first12))
    return (10 - total % 10) % 10


def ean13(first12: str) -> str:
    return first12 + str(ean13_check_digit(first12))


def ean13_modules(code: str) -> str:
    """The 95-module bar pattern ('1' = bar) of a complete EAN-13 code."""
    if len(code) != 13 or not code.isdigit() or ean13_check_digit(code[:12]) != int(code[12]):
        raise ValueError(f"not a valid EAN-13: {code}")
    parity = _PARITY[int(code[0])]
    left = "".join(
        (_L if p == "L" else _G)[int(d)] for p, d in zip(parity, code[1:7], strict=True)
    )
    right = "".join(_R[int(d)] for d in code[7:])
    return "101" + left + "01010" + right + "101"


# ---- catalogue -------------------------------------------------------------------------------


def _name(rng: random.Random) -> str:
    return rng.choice(BRAND_START) + rng.choice(BRAND_END)


def build_catalogue() -> list[Spec]:
    """Two families per shape; each family is a look-alike pair (same layout, other flavour)."""
    specs: list[Spec] = []
    index = 0
    for shape in ("can", "small_bottle", "large_bottle", "contour_bottle"):
        for family in range(2):
            rng = random.Random(f"{shape}/{family}")
            brand = _name(rng)
            kind = rng.choice(KINDS)
            hue = rng.random()
            layout_seed = rng.randrange(1 << 30)
            flavours = rng.sample(FLAVOURS, 2)
            accents = (rng.random(), (hue + 0.5 + 0.08 * rng.random()) % 1.0)
            for member in range(2):
                index += 1
                specs.append(
                    Spec(
                        sku=f"GEN-{index:03d}",
                        shape=shape,
                        geometry_key=f"{shape.replace('_', '.')}.gen{index:03d}",
                        seed=rng.randrange(1 << 30),
                        brand=brand,
                        flavour=flavours[member],
                        kind=kind,
                        hue=(hue + 0.015 * member) % 1.0,
                        accent_hue=accents[member],
                        layout_seed=layout_seed,
                        pair=f"{shape}-{family}",
                    )
                )
    # Boxes are appended so the 16 SKUs above keep their numbers. A look-alike pair of 200 mL
    # cartons, one 1 L carton, and a slim box with no barcode panel at all.
    for family, (variant, members, barcode) in enumerate(
        (("carton_200", 2, True), ("carton_1000", 1, True), ("slim_box", 1, False))
    ):
        rng = random.Random(f"box/{family}")
        brand = _name(rng)
        kind = rng.choice(SLIM_KINDS if variant == "slim_box" else BOX_KINDS)
        hue = rng.random()
        layout_seed = rng.randrange(1 << 30)
        flavours = rng.sample(FLAVOURS, 2)
        accents = (rng.random(), (hue + 0.5 + 0.08 * rng.random()) % 1.0)
        for member in range(members):
            index += 1
            specs.append(
                Spec(
                    sku=f"GEN-{index:03d}",
                    shape="box",
                    geometry_key=f"box.gen{index:03d}",
                    seed=rng.randrange(1 << 30),
                    brand=brand,
                    flavour=flavours[member],
                    kind=kind,
                    hue=(hue + 0.015 * member) % 1.0,
                    accent_hue=accents[member],
                    layout_seed=layout_seed,
                    pair=f"box-{family}",
                    variant=variant,
                    barcode=barcode,
                )
            )
    return specs


def gtin_for(spec: Spec) -> str | None:
    """The product's EAN-13, or None for a SKU that carries no barcode."""
    # 200-299 is the GS1 range reserved for in-store/restricted use, so no real product shares it.
    if not spec.barcode:
        return None
    number = int(spec.sku.split("-")[1])
    return ean13(f"2000000{number:05d}")


# ---- rendering -------------------------------------------------------------------------------


def _hsl(h: float, s: float, light: float) -> tuple[int, int, int]:
    r, g, b = colorsys.hls_to_rgb(h % 1.0, light, s)
    return int(r * 255), int(g * 255), int(b * 255)


def _font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.load_default(size=size)


def _text(draw: ImageDraw.ImageDraw, xy, text, size, fill, bold=False, anchor="la") -> None:
    draw.text(
        xy,
        text,
        font=_font(size),
        fill=fill,
        anchor=anchor,
        stroke_width=1 if bold else 0,
        stroke_fill=fill,
    )


def _wrap(draw, text, size, width) -> list[str]:
    words, lines, line = text.split(), [], ""
    for word in words:
        trial = f"{line} {word}".strip()
        if draw.textlength(trial, font=_font(size)) <= width or not line:
            line = trial
        else:
            lines.append(line)
            line = word
    if line:
        lines.append(line)
    return lines


def _logo(draw, rng, cx, cy, r, palette) -> None:
    """A random geometric mark: a few overlapping primitives, no resemblance intended."""
    for _ in range(rng.randint(3, 5)):
        colour = rng.choice(palette)
        kind = rng.choice(("circle", "tri", "hex", "bar", "ring"))
        ox, oy = rng.uniform(-0.45, 0.45) * r, rng.uniform(-0.45, 0.45) * r
        s = rng.uniform(0.35, 0.8) * r
        x, y = cx + ox, cy + oy
        if kind == "circle":
            draw.ellipse((x - s, y - s, x + s, y + s), fill=colour)
        elif kind == "ring":
            draw.ellipse((x - s, y - s, x + s, y + s), outline=colour, width=max(4, int(s * 0.18)))
        elif kind == "tri":
            draw.polygon([(x, y - s), (x - s, y + s * 0.8), (x + s, y + s * 0.8)], fill=colour)
        elif kind == "hex":
            draw.regular_polygon((x, y, s), 6, rotation=rng.randrange(60), fill=colour)
        else:
            draw.rectangle((x - s, y - s * 0.25, x + s, y + s * 0.25), fill=colour)


def _background(draw, rng, w, h, base, accent, light) -> None:
    draw.rectangle((0, 0, w, h), fill=base)
    style = rng.choice(("stripes", "dots", "waves"))
    step = int(h * rng.uniform(0.07, 0.12))
    if style == "stripes":
        for x in range(-h, w, step * 2):
            draw.polygon(
                [(x, 0), (x + step, 0), (x + step + h // 2, h), (x + h // 2, h)], fill=accent
            )
    elif style == "dots":
        for y in range(step // 2, h, step):
            for x in range(step // 2 + (y // step % 2) * step // 2, w, step):
                draw.ellipse(
                    (x - step // 6, y - step // 6, x + step // 6, y + step // 6), fill=light
                )
    else:
        for k in range(6):
            y0 = h * (0.15 + 0.14 * k)
            pts = [
                (x, y0 + 0.04 * h * __import__("math").sin(x / (w / 14) + k))
                for x in range(0, w + 1, 12)
            ]
            draw.line(pts, fill=accent if k % 2 else light, width=max(3, h // 70))


def _barcode(draw, code, x, y, w, h) -> None:
    modules = ean13_modules(code)
    quiet = 9
    unit = w / (len(modules) + 2 * quiet)
    draw.rectangle((x, y, x + w, y + h), fill=(255, 255, 255))
    for i, bit in enumerate(modules):
        if bit == "1":
            guard = i < 3 or 45 <= i < 50 or i >= 92
            bottom = y + h - (0 if guard else h * 0.16) - h * 0.14
            draw.rectangle(
                (x + (quiet + i) * unit, y + h * 0.06, x + (quiet + i + 1) * unit - 0.5, bottom),
                fill=(10, 10, 10),
            )
    size = max(8, int(h * 0.13))
    _text(draw, (x + quiet * unit * 0.4, y + h * 0.88), code[0], size, (10, 10, 10))
    _text(
        draw,
        (x + (quiet + 3 + 21) * unit, y + h * 0.88),
        code[1:7],
        size,
        (10, 10, 10),
        anchor="ma",
    )
    _text(
        draw,
        (x + (quiet + 50 + 21) * unit, y + h * 0.88),
        code[7:],
        size,
        (10, 10, 10),
        anchor="ma",
    )


def _recycle(draw, cx, cy, r, colour) -> None:
    for k in range(3):
        draw.arc(
            (cx - r, cy - r, cx + r, cy + r),
            40 + k * 120,
            130 + k * 120,
            fill=colour,
            width=max(3, int(r * 0.22)),
        )
    draw.regular_polygon((cx, cy, r * 0.35), 3, fill=colour)


def _front(draw, spec, rng, box, palette, volume, size_scale) -> None:
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    cx = x0 + w / 2
    ink, soft = palette["ink"], palette["paper"]
    _logo(
        draw,
        random.Random(spec.layout_seed),
        cx,
        y0 + h * 0.2,
        h * 0.12,
        [palette["accent"], palette["light"], soft],
    )
    brand_size = int(h * 0.15 * size_scale)
    while draw.textlength(spec.brand, font=_font(brand_size)) > w * 0.9:
        brand_size -= 2
    _text(draw, (cx, y0 + h * 0.44), spec.brand, brand_size, soft, bold=True, anchor="mm")
    flav_size = int(h * 0.085 * size_scale)
    while draw.textlength(spec.flavour.upper(), font=_font(flav_size)) > w * 0.85:
        flav_size -= 2
    pad = int(flav_size * 0.35)
    tw = draw.textlength(spec.flavour.upper(), font=_font(flav_size))
    draw.rounded_rectangle(
        (
            cx - tw / 2 - pad,
            y0 + h * 0.62 - flav_size * 0.75 - pad / 2,
            cx + tw / 2 + pad,
            y0 + h * 0.62 + flav_size * 0.75 + pad / 2,
        ),
        radius=pad,
        fill=palette["accent"],
    )
    _text(draw, (cx, y0 + h * 0.62), spec.flavour.upper(), flav_size, ink, bold=True, anchor="mm")
    _text(draw, (cx, y0 + h * 0.76), spec.kind, int(h * 0.06 * size_scale), soft, anchor="mm")
    _text(draw, (cx, y0 + h * 0.86), volume, int(h * 0.05 * size_scale), soft, anchor="mm")


def _back(draw, spec, rng, box, palette, code, size_scale, ingredient_pool=INGREDIENTS) -> None:
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    draw.rectangle(box, fill=palette["paper"])
    ink = (30, 30, 34)
    pad = w * 0.05
    y = y0 + pad
    body = max(10, int(h * 0.029 * size_scale))
    _text(draw, (x0 + pad, y), spec.brand + " " + spec.flavour, int(body * 1.5), ink, bold=True)
    y += body * 2.3
    ingredients = ", ".join(rng.sample(ingredient_pool, 6))
    for line in _wrap(draw, "INGREDIENTS: " + ingredients + ".", body, w - 2 * pad):
        _text(draw, (x0 + pad, y), line, body, ink)
        y += body * 1.25
    y += body * 0.5
    # Nutrition table.
    rows = (
        ("Energy", f"{rng.randint(15, 190)} kJ"),
        ("Fat", "0 g"),
        ("Carbohydrate", f"{rng.uniform(0, 11):.1f} g"),
        ("  of which sugars", f"{rng.uniform(0, 10):.1f} g"),
        ("Protein", "0 g"),
        ("Salt", f"{rng.uniform(0, 0.1):.2f} g"),
    )
    row_h = body * 1.35
    top = y
    draw.rectangle((x0 + pad, top, x1 - pad, top + row_h * (len(rows) + 1)), outline=ink, width=2)
    _text(draw, (x0 + pad * 1.4, top + row_h * 0.15), "Nutrition per 100 mL", body, ink, bold=True)
    for i, (k, v) in enumerate(rows, start=1):
        yy = top + row_h * i
        draw.line((x0 + pad, yy, x1 - pad, yy), fill=ink, width=1)
        _text(draw, (x0 + pad * 1.4, yy + row_h * 0.15), k, body, ink)
        _text(draw, (x1 - pad * 1.4, yy + row_h * 0.15), v, body, ink, anchor="ra")
    y = top + row_h * (len(rows) + 1) + body
    code_top = y1 - h * 0.25 - pad
    for line in _wrap(draw, " ".join(rng.sample(DISCLAIMERS, 2)), int(body * 0.9), w - 2 * pad):
        if y + body * 1.15 > code_top - body * 3:
            break
        _text(draw, (x0 + pad, y), line, int(body * 0.9), ink)
        y += body * 1.15
    _recycle(draw, x1 - pad - body * 1.8, code_top - body * 1.9, body * 1.4, ink)
    if code is not None:
        _barcode(draw, code, x0 + pad, code_top, w - 2 * pad, h * 0.25)
    else:
        # A product with no barcode panel: a batch line in the space the code would take, so the
        # back is not blank where a reader looks for it.
        _text(draw, (x0 + pad, code_top + body * 0.5), "Batch and date: see top", body, ink)


def render(spec: Spec) -> Image.Image:
    shape = SHAPES[spec.shape]
    circumference = 2 * 3.141592653589793 * shape["radius"]
    width = TEXTURE_WIDTH
    height = round(width * shape["height"] / circumference)
    W, H = width * SUPERSAMPLE, height * SUPERSAMPLE
    img = Image.new("RGB", (W, H))
    draw = ImageDraw.Draw(img)
    rng = random.Random(spec.layout_seed)
    base = _hsl(spec.hue, 0.62, 0.38)
    accent = _hsl(spec.accent_hue, 0.7, 0.55)
    light = _hsl(spec.hue, 0.5, 0.7)
    palette = {
        "base": base,
        "accent": accent,
        "light": light,
        "ink": (24, 22, 26),
        "paper": (244, 240, 232),
    }
    band_lo, band_hi = shape["band"]
    y_lo, y_hi = int(H * band_lo), int(H * band_hi)
    # Bottle plastic outside the label band: a pale tint with a soft highlight column.
    if band_lo > 0:
        tint = _hsl(spec.hue, 0.25, 0.82)
        draw.rectangle((0, 0, W, H), fill=tint)
    label = Image.new("RGB", (W, y_hi - y_lo))
    ld = ImageDraw.Draw(label)
    lw, lh = label.size
    _background(ld, rng, lw, lh, base, accent, light)
    # Panels around the circumference: front 0.05-0.47, side 0.47-0.53, back 0.53-0.95.
    _front(
        ld,
        spec,
        rng,
        (int(lw * 0.05), 0, int(lw * 0.47), lh),
        palette,
        shape["volume"],
        1.0 if band_lo == 0 else 0.95,
    )
    _back(
        ld,
        spec,
        random.Random(spec.seed),
        (int(lw * 0.53), int(lh * 0.04), int(lw * 0.95), int(lh * 0.96)),
        palette,
        gtin_for(spec),
        1.0,
    )
    img.paste(label, (0, y_lo))
    img = img.resize((width, height), Image.LANCZOS)
    return img


def _fit(draw, xy, text, max_width, size, fill, bold=False, anchor="la") -> int:
    """Draw ``text`` within ``max_width``, shrinking from ``size``; returns the size used."""
    size = max(int(size), 8)
    while size > 8 and draw.textlength(text, font=_font(size)) > max_width:
        size -= 2
    _text(draw, xy, text, size, fill, bold=bold, anchor=anchor)
    return size


def _side_right(draw, spec, face, palette, volume) -> None:
    """Right face: brand, flavour tag and volume. Text is horizontal and sized to the width."""
    w, h = face.size
    soft, ink = palette["paper"], palette["ink"]
    cx = w / 2
    _logo(
        draw,
        random.Random(spec.layout_seed + 1),
        cx,
        h * 0.14,
        w * 0.28,
        [palette["accent"], palette["light"], soft],
    )
    _fit(draw, (cx, h * 0.34), spec.brand, w * 0.86, h * 0.075, soft, True, "mm")
    pad = w * 0.06
    size = _fit(draw, (cx, h * 0.5), spec.flavour.upper(), w * 0.8, h * 0.05, ink, True, "mm")
    tw = draw.textlength(spec.flavour.upper(), font=_font(size))
    draw.rounded_rectangle(
        (cx - tw / 2 - pad, h * 0.5 - size * 0.8, cx + tw / 2 + pad, h * 0.5 + size * 0.8),
        radius=pad,
        fill=palette["accent"],
    )
    _text(draw, (cx, h * 0.5), spec.flavour.upper(), size, ink, bold=True, anchor="mm")
    _fit(draw, (cx, h * 0.62), spec.kind, w * 0.86, h * 0.04, soft, False, "mm")
    _fit(draw, (cx, h * 0.7), volume, w * 0.86, h * 0.05, soft, True, "mm")


def _side_left(draw, spec, face, rng, palette) -> None:
    """Left face: paper panel with the small print."""
    w, h = face.size
    ink = (30, 30, 34)
    draw.rectangle((0, 0, w, h), fill=palette["paper"])
    pad = w * 0.08
    body = max(10, int(min(h * 0.026, w * 0.07)))
    y = h * 0.06
    for line in _wrap(draw, " ".join(rng.sample(DISCLAIMERS, 3)), body, w - 2 * pad):
        _text(draw, (pad, y), line, body, ink)
        y += body * 1.3
    _recycle(draw, w / 2, h * 0.86, min(w * 0.18, h * 0.05), ink)
    _fit(draw, (w / 2, h * 0.94), "Please recycle", w - 2 * pad, body, ink, False, "mm")


def _top(draw, spec, face, rng, palette) -> None:
    """Top face: crimped fin lines at the front and back, brand and a date code between."""
    w, h = face.size
    ink = (24, 22, 26)
    draw.rectangle((0, 0, w, h), fill=palette["base"])
    fin = h * 0.16
    step = max(6, int(w * 0.03))
    for edge in (0, h - fin):
        draw.rectangle((0, edge, w, edge + fin), fill=palette["light"])
        for x in range(0, w, step * 2):
            draw.rectangle((x, edge, x + step, edge + fin), fill=palette["accent"])
    cx = w / 2
    _fit(draw, (cx, h * 0.38), spec.brand, w * 0.8, h * 0.18, palette["paper"], True, "mm")
    date = f"BEST BEFORE {rng.randint(1, 28):02d}.{rng.randint(1, 12):02d}.{rng.randint(27, 29)}"
    _fit(draw, (cx, h * 0.62), date, w * 0.86, h * 0.1, ink, False, "mm")
    _fit(
        draw,
        (cx, h * 0.76),
        f"LOT {rng.randint(10000, 99999)}",
        w * 0.7,
        h * 0.08,
        ink,
        False,
        "mm",
    )


def box_faces(spec: Spec) -> dict:
    """Pixel rectangle and frame of each face, plus the atlas size, for this box SKU."""
    variant = BOX_VARIANTS[spec.variant]
    layout = box_layout(variant["size"])
    return {
        "atlas": layout["atlas"],
        "faces": {
            name: {
                "rect_px": list(layout["rects"][name]),
                "normal": list(BOX_FACE_FRAMES[name][0]),
                "image_right": list(BOX_FACE_FRAMES[name][1]),
                "image_up": list(BOX_FACE_FRAMES[name][2]),
            }
            for name in BOX_FACES
        },
    }


def render_box(spec: Spec) -> Image.Image:
    """The box's atlas: one upright, unmirrored panel per face (see BOX_FACE_FRAMES)."""
    variant = BOX_VARIANTS[spec.variant]
    layout = box_layout(variant["size"])
    width, height = layout["atlas"]
    base = _hsl(spec.hue, 0.62, 0.38)
    accent = _hsl(spec.accent_hue, 0.7, 0.55)
    light = _hsl(spec.hue, 0.5, 0.7)
    palette = {
        "base": base,
        "accent": accent,
        "light": light,
        "ink": (24, 22, 26),
        "paper": (244, 240, 232),
    }
    atlas = Image.new("RGB", (width * SUPERSAMPLE, height * SUPERSAMPLE), base)
    for name in BOX_FACES:
        x, y, w, h = (v * SUPERSAMPLE for v in layout["rects"][name])
        face = Image.new("RGB", (w, h), base)
        draw = ImageDraw.Draw(face)
        if name == "front":
            _background(draw, random.Random(spec.layout_seed), w, h, base, accent, light)
            _front(
                draw,
                spec,
                None,
                (int(w * 0.04), 0, int(w * 0.96), h),
                palette,
                variant["volume"],
                1.0,
            )
        elif name == "right":
            _background(draw, random.Random(spec.layout_seed), w, h, base, accent, light)
            _side_right(draw, spec, face, palette, variant["volume"])
        elif name == "back":
            code = gtin_for(spec)
            _back(
                draw,
                spec,
                random.Random(spec.seed),
                (0, int(h * 0.03), w, int(h * 0.97)),
                palette,
                code,
                1.0,
                BOX_INGREDIENTS,
            )
        elif name == "left":
            _side_left(draw, spec, face, random.Random(spec.seed + 1), palette)
        elif name == "top":
            _top(draw, spec, face, random.Random(spec.seed + 2), palette)
        atlas.paste(face, (x, y))
    return atlas.resize((width, height), Image.LANCZOS)


# ---- manifest --------------------------------------------------------------------------------


def manifest_text(specs: list[Spec]) -> str:
    lines = [
        "# Generated by tools/label_factory.py. Do not edit by hand.",
        "# Every product is invented; GTINs sit in the GS1 restricted range (prefix 2).",
        "schema_version: 1",
        "products:",
    ]
    for s in specs:
        lines.append(f"  - sku: {s.sku}")
        if gtin_for(s) is not None:
            lines.append(f'    gtin: "{gtin_for(s)}"')
        lines += [
            f"    brand: {s.brand}",
            f"    flavour: {s.flavour}",
            f"    kind: {s.kind}",
            f"    shape: {s.shape}",
            f"    geometry_key: {s.geometry_key}",
            f"    texture: {s.texture}",
            f"    texture_rotation_deg: {s.texture_rotation_deg}",
            f"    lookalike_group: {s.pair}",
        ]
        if s.is_box:
            variant = BOX_VARIANTS[s.variant]
            layout = box_faces(s)
            lines += [
                f"    box_variant: {s.variant}",
                f"    visual_mesh: {box_mesh_name(s.variant)}",
                f"    size_xyz_m: {list(variant['size'])}",
                f"    mass_kg: {variant['mass_kg']}",
                f"    barcode: {'true' if s.barcode else 'false'}",
                f"    atlas_px: {list(layout['atlas'])}",
                "    faces:",
            ]
            for name in BOX_FACES:
                face = layout["faces"][name]
                lines.append(
                    f"      {name}: {{rect_px: {face['rect_px']}, normal: {face['normal']}, "
                    f"image_right: {face['image_right']}, image_up: {face['image_up']}}}"
                )
    return "\n".join(lines) + "\n"


def box_mesh_name(variant: str) -> str:
    return f"box_{variant}.obj"


def catalog_block(specs: list[Spec]) -> str:
    """Catalogue entries, one geometry_key per SKU.

    The key is the catalogue's primary key in four loaders, so each generated SKU gets its own key
    with the shape of its class (the can.citrus precedent); class_fallback stays with the original
    entry of each class.
    """
    lines = [CATALOG_BEGIN]
    for s in specs:
        if s.is_box:
            size = list(BOX_VARIANTS[s.variant]["size"])
            lines += [
                f"  - geometry_key: {s.geometry_key}",
                "    product_class: box",
                f"    sku: {s.sku}",
                "    class_fallback: false",
                "    shape:",
                "      type: box",
                f"      size_xyz_m: {size}",
            ]
            continue
        shape = SHAPES[s.shape]
        lines += [
            f"  - geometry_key: {s.geometry_key}",
            f"    product_class: {s.shape}",
            f"    sku: {s.sku}",
            "    class_fallback: false",
            "    shape:",
            "      type: cylinder",
            f"      radius_m: {shape['radius']}",
            f"      height_m: {shape['height']}",
        ]
    lines.append(CATALOG_END)
    return "\n".join(lines) + "\n"


def catalog_text(specs: list[Spec]) -> str:
    text = CATALOG.read_text()
    block = catalog_block(specs)
    if CATALOG_BEGIN in text:
        head, rest = text.split(CATALOG_BEGIN, 1)
        tail = rest.split(CATALOG_END, 1)[1].lstrip("\n")
        return head + block + tail
    return text.rstrip("\n") + "\n" + block


def png_bytes(image: Image.Image) -> bytes:
    import io

    buf = io.BytesIO()
    image.save(buf, format="PNG", optimize=False, compress_level=9)
    return buf.getvalue()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--check", action="store_true", help="fail if any output would change")
    args = parser.parse_args(argv)
    specs = build_catalogue()
    outputs = {MANIFEST: manifest_text(specs).encode(), CATALOG: catalog_text(specs).encode()}
    for spec in specs:
        image = render_box(spec) if spec.is_box else render(spec)
        outputs[TEXTURE_DIR / Path(spec.texture).name] = png_bytes(image)
    stale = []
    for path, data in outputs.items():
        if not path.exists() or path.read_bytes() != data:
            stale.append(path)
            if not args.check:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
    for path in stale:
        print(("would change " if args.check else "wrote ") + str(path.relative_to(ROOT)))
    return 1 if (args.check and stale) else 0


if __name__ == "__main__":
    sys.exit(main())
