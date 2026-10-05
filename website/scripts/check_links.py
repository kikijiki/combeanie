#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check generated HTML routes, fragments and local assets, including raw component links."""

from html.parser import HTMLParser
import os
from pathlib import Path
import sys
from urllib.parse import unquote, urljoin, urlsplit


class Page(HTMLParser):
    def __init__(self, source):
        super().__init__()
        self.ids = set()
        self.links = []
        self.feed(source)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if "id" in attrs:
            self.ids.add(attrs["id"])
        for name in ("href", "src", "poster"):
            if attrs.get(name):
                self.links.append(attrs[name])


def check(build, base):
    pages = {p: Page(p.read_text()) for p in build.rglob("*.html")}
    if not pages:
        return [f"{build}: no generated HTML; run bun run build first"], 0
    errors = []
    for path, page in pages.items():
        relative = path.relative_to(build).as_posix()
        route = base + (relative[:-10] if relative.endswith("index.html") else relative)
        for link in page.links:
            if urlsplit(link).scheme or link.startswith("//"):
                continue
            parsed = urlsplit(urljoin(route, link))
            target_url = unquote(parsed.path)
            if not target_url.startswith(base):
                errors.append(f"{relative}: outside base URL {base}: {link}")
                continue
            target = build / target_url[len(base) :]
            candidates = [target, target / "index.html", target.with_suffix(".html")]
            target = next((p for p in candidates if p.is_file()), None)
            if target is None:
                errors.append(f"{relative}: missing target: {link}")
            elif (
                parsed.fragment
                and target in pages
                and unquote(parsed.fragment) not in pages[target].ids
            ):
                errors.append(f"{relative}: missing fragment: {link}")
    return sorted(set(errors)), len(pages)


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1] / "build"
    base_url = os.environ.get("DOCS_BASE_URL", "/")
    if not base_url.startswith("/") or not base_url.endswith("/"):
        sys.exit("DOCS_BASE_URL must begin and end with /")
    errors, count = check(root, base_url)
    print(
        "\n".join(errors)
        if errors
        else f"Checked {count} HTML pages under {base_url}: routes, fragments and assets resolve"
    )
    sys.exit(bool(errors))
