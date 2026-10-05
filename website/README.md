# combeanie website

Docusaurus public manual: introduction, running the system, fundamentals, architecture and reference.
No public hosting destination is established by a successful local build.

## Reproducible development

From the repository root, enter `nix develop .#docs` (or the default ROS shell). `flake.lock`
pins Bun and Node; `website/bun.lock` pins JavaScript dependencies. No host-global tools are needed.

```bash
nix develop .#docs --command just docs       # frozen install, typecheck, build, local links
nix develop .#docs --command just docs-serve # frozen install, live server at localhost:3000
# LAN binding / alternate port:
nix develop .#docs --command just docs-serve --host 0.0.0.0 --port 3100
```

Inside that shell, individual checks are `cd website`, `bun install --frozen-lockfile`,
`bun run typecheck`, `bun run build`, and `bun run check-links`.
The checked-in CI workflow includes these validations; repository CI enablement is an external setting.
`nix flake check` covers robotics/repository checks, while `just docs` is the website gate.

## Hosting paths

`DOCS_SITE_URL` sets the canonical origin and `DOCS_BASE_URL` the root or project prefix
(with leading and trailing slashes). These configure a build; they do not publish it.

```bash
DOCS_BASE_URL=/combeanie/ nix develop .#docs --command just docs
# Keep the same variables when serving or checking this output:
DOCS_BASE_URL=/combeanie/ nix develop .#docs --command bash -c 'cd website && bun run serve --host 127.0.0.1'
```

The link check covers generated HTML href/src/poster URLs and local fragments, including component
links that the Markdown build checker cannot see. It does not check external sites or prove WebGL support.
Check diagram pages at 320/375/390 px, glossary keyboard controls, and the Introduction with
JavaScript disabled, WebGL unavailable, and a blocked GLB request when changing visual components.
Models load only after an explicit button press; static schematics remain available without them.

## Demo video

`static/media/restocking-demo.mp4` remains labelled historical ground-truth footage. To re-record,
follow [`scripts/record-demo.md`](scripts/record-demo.md) in a separately authorized, serialized
simulator session. A docs build never runs the robot or overwrites media.
