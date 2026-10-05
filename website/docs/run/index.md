---
id: index
title: How to run
sidebar_position: 1
---

# How to run

Combeanie is a Nix-pinned ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2 simulation.
The supported operator surface is the root **`justfile`**.

| Intent | Command | Display? |
| --- | --- | --- |
| Watch autonomous restock | [`just demo`](./demo.md) | Required |
| Interactive full stack | [`just launch-baseline`](./scenarios.md) | Required by default |
| Physics + controllers | [`just launch-sim`](./scenarios.md) | Required by default |
| Robot model only | [`just launch-rviz`](./scenarios.md) | Required by default |
| Headless MoveIt smoke | `just planning-smoke` | No |
| Measure failures | [`just benchmark`](./tests-and-benchmarks.md) | No (needs EGL/OpenGL) |

## First-time setup

Install Nix with flakes and direnv, then from the repository root:

```bash
direnv allow
just doctor
just build
just test
```

Without direnv:

```bash
nix develop .
just doctor
```

Use `nix develop .`, **not** `nix develop path:.`. The `path:` form copies the working
tree into the Nix store (including `ros_ws/build`) and is slow or fails mid-copy.

No host ROS, Gazebo, or MoveIt install is required. The flake pins the toolchain.

Public docs (this site) build with Bun under `website/` (`just docs`). Internal engineering
docs are kept privately.

### Shell profiles

| Profile | Enter | Contents |
| --- | --- | --- |
| `baseline` (default) | `direnv allow` / `nix develop .` | Viz + Gazebo + ros2_control + MoveIt |
| `simulation` | `nix develop .#simulation` | Omits MoveIt (`launch-sim`, `control-smoke`) |
| `foundation` | `nix develop .#foundation` | Description / smaller set |

### What `just doctor` checks

`scripts/doctor.bash` fails on missing flake shell, wrong `ROS_DISTRO`, missing toolchain
commands, or missing ROS packages for the active profile. Missing `DISPLAY` /
`WAYLAND_DISPLAY` is a **warning**; GUI launches still die without one.

## Build and test

```bash
just build
just test
just build-package restocker_world_state
just test-package restocker_world_state
just clean          # ros_ws/{build,install,log}
just clean-ros-logs # ~/.ros/log, opt-in, outside the repo
```

`just jobs=2 build` and `just jobs=2 build-package restocker_world_state` run one package
at a time with at most two compiler jobs in total. The default is eight; `jobs` must be a
positive integer. The recipes set both `CMAKE_BUILD_PARALLEL_LEVEL` and `MAKEFLAGS` so
colcon cannot replace the requested limit with the host CPU count. This bounds scheduled
compiler jobs, not all subprocess threads or other work running on the machine.

Earlier recipes set only the CMake variable and allowed multiple packages at once. With
the pinned colcon version, that requested limit was not enforced when `MAKEFLAGS` was unset.
Historical build durations and machine-load measurements retain their original conditions;
the command's `jobs` value alone does not establish their effective concurrency or quietness.

Before review: `just format && just lint && just test && just flake-check`.

### Optional: kache build cache

If [`kache`](https://github.com/kunobi-ninja/kache) is installed and on `PATH`, `just build` and
`just build-package` route every C/C++ compile through it via CMake's compiler-launcher mechanism
(`CMAKE_C_COMPILER_LAUNCHER` / `CMAKE_CXX_COMPILER_LAUNCHER`) — a content-addressed cache that
replays unchanged compiles from `~/.cache/kache` instead of re-running the compiler. Nothing
changes without it: if `kache` is absent, the build command is exactly the same as without the
feature, and the Nix sandbox build (`just flake-check`) never sees kache — no kache, no daemon.

Things worth knowing:

- **Switching the launcher on or off (or the first build after this feature lands) reconfigures
  and fully rebuilds existing build directories once** — colcon re-runs CMake whenever
  `--cmake-args` change. After that, incremental builds behave exactly as before.
- **Verify interception** with `kache stats` before and after a build: misses rise on a cold
  build, hits appear on a rebuild from a fresh shell at the same path. `kache doctor` checks the
  installation; `kache stats` reports the daemon and the hit rate.
- **Cache keys include GCC's `-frandom-seed`, and the flake pins it once for every shell.** The
  Nix dev shell (direnv or `nix develop`) exports `NIX_CFLAGS_COMPILE` with
  `-frandom-seed=<id>`; the gcc wrapper injects it into every compile and kache includes the
  resolved flag in its key. By default `<id>` is derived from the dev-shell profile's store path,
  so it changed whenever the profile was rebuilt and every fresh shell started from a cold cache.
  The flake overrides that at the source: `flake.nix`'s shared `mkShell` sets
  `NIX_OUTPATH_USED_AS_RANDOM_SEED = "combeanie"`, which stdenv's `reproducible-builds.sh` setup
  hook reads in place of `$out`, so every dev shell exports the constant
  `-frandom-seed=combeanie` — no build-time rewriting involved, and keys stay identical across
  shells and profiles. Do **not** pin the flag on the command line instead: kache classifies an
  explicit `-frandom-seed` in the compile arguments as unsupported and routes the compile around
  the cache entirely (passthrough, zero reuse).
- **Paths are normalized only down to the workspace root**, so a *differently named* build or
  install directory below it (`ros_ws/build` vs `ros_ws/build-b`, `install` vs `install-b`)
  still misses on every TU that references those absolute paths — rosidl-generated sources and
  their `-I`/`-c` paths, and cross-package `-isystem …/install/…` includes. Rebuilds in the
  *same* path hit; do not expect reuse across separately named build trees.
- Objects served from the cache are byte-identical to the ones the original compile produced.
  Their debug-info paths are directory-agnostic: kache normalizes `DW_AT_comp_dir` to
  `/proc/self/cwd` (in-workspace) or `/kache/cc-root`, so a cached object restored into a
  differently named build directory records neither path (plain, non-kache compiles record the
  real working directory).

## Display requirement

Gazebo's client is a *required* process when `gui:=true`. Without a display, the Qt platform
plugin fails within seconds and takes the composition down. Over SSH or in CI, pass
`gui:=false` (and `rviz:=false` for baseline) or use headless recipes (`planning-smoke`,
`benchmark`).

## Next

- [Demo](./demo.md): fastest way to watch the autonomous survey/restock campaign
- [Scenarios and parameters](./scenarios.md): every launch mode and argument table
- [Tests and benchmarks](./tests-and-benchmarks.md): acceptance tests and measurement
