# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Plot benchmark results as SVG, using nothing but the standard library."""
# The pinned flake has no matplotlib; an SVG bar chart is small, renders in a browser and diffs as
# text.

from __future__ import annotations

from dataclasses import dataclass
import html

# One accent for the measured quantity, one for failure rows. Two keeps charts readable in print.
_BAR_FILL = "#33618f"
_BAR_FILL_ALTERNATE = "#9c4221"
_AXIS = "#4a4a4a"
_LABEL = "#1c1c1c"

_ROW_HEIGHT = 26
_LABEL_WIDTH = 330
_BAR_WIDTH = 380
_VALUE_WIDTH = 110
_TOP = 62
_BOTTOM = 34


@dataclass(frozen=True)
class Bar:
    """One row of a horizontal bar chart."""

    label: str
    value: float
    # Rendered value text, so a count and a duration can share the renderer.
    annotation: str
    highlight: bool = False


def horizontal_bars(title: str, subtitle: str, bars: list[Bar]) -> str:
    """Render a horizontal bar chart as a standalone SVG document."""
    width = _LABEL_WIDTH + _BAR_WIDTH + _VALUE_WIDTH + 24
    height = _TOP + _BOTTOM + _ROW_HEIGHT * max(1, len(bars))
    largest = max((bar.value for bar in bars), default=0.0)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="DejaVu Sans, Verdana, sans-serif">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        f'<text x="16" y="26" font-size="15" font-weight="bold" fill="{_LABEL}">'
        f"{html.escape(title)}</text>",
        f'<text x="16" y="45" font-size="11" fill="{_AXIS}">{html.escape(subtitle)}</text>',
    ]
    if not bars:
        parts.append(
            f'<text x="16" y="{_TOP + 16}" font-size="12" fill="{_AXIS}">no data</text></svg>'
        )
        return "".join(parts)

    axis_x = _LABEL_WIDTH + 8
    for index, bar in enumerate(bars):
        top = _TOP + index * _ROW_HEIGHT
        centre = top + _ROW_HEIGHT / 2
        length = 0.0 if largest <= 0 else (bar.value / largest) * _BAR_WIDTH
        fill = _BAR_FILL_ALTERNATE if bar.highlight else _BAR_FILL
        label = bar.label if len(bar.label) <= 52 else bar.label[:49] + "..."
        parts.append(
            f'<text x="{_LABEL_WIDTH}" y="{centre + 4:.1f}" font-size="11" text-anchor="end" '
            f'fill="{_LABEL}">{html.escape(label)}</text>'
        )
        parts.append(
            f'<rect x="{axis_x}" y="{top + 5}" width="{max(length, 1.0):.1f}" '
            f'height="{_ROW_HEIGHT - 10}" fill="{fill}" rx="2"/>'
        )
        parts.append(
            f'<text x="{axis_x + max(length, 1.0) + 8:.1f}" y="{centre + 4:.1f}" font-size="11" '
            f'fill="{_AXIS}">{html.escape(bar.annotation)}</text>'
        )
    baseline = _TOP + len(bars) * _ROW_HEIGHT
    parts.append(
        f'<line x1="{axis_x}" y1="{_TOP}" x2="{axis_x}" y2="{baseline}" '
        f'stroke="{_AXIS}" stroke-width="1"/>'
    )
    parts.append("</svg>")
    return "".join(parts)


def histogram(title: str, subtitle: str, values: list[float], unit: str, buckets: int = 8) -> str:
    """Render a histogram of ``values`` as a horizontal bar chart."""
    if not values:
        return horizontal_bars(title, subtitle, [])
    low = min(values)
    high = max(values)
    if high <= low:
        return horizontal_bars(
            title,
            subtitle,
            [Bar(f"{low:.1f} {unit}", float(len(values)), f"{len(values)}")],
        )
    span = (high - low) / buckets
    counts = [0] * buckets
    for value in values:
        index = min(buckets - 1, int((value - low) / span))
        counts[index] += 1
    bars = [
        Bar(
            f"{low + index * span:.0f}-{low + (index + 1) * span:.0f} {unit}",
            float(count),
            str(count),
        )
        for index, count in enumerate(counts)
    ]
    return horizontal_bars(title, subtitle, bars)
