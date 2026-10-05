# Contributing

## Workflow

1. Enter the pinned environment with `direnv allow` or `nix develop .`.
2. Run `just doctor` before diagnosing build failures.
3. Add unit and integration tests with implementation changes.
4. Run `just format`, `just lint`, `just test`, and `just flake-check` before requesting review.

Keep changes incremental and keep the tree buildable and runnable. Do not add host-global setup, a top-level Makefile, CUDA to the baseline shell, or model services as deterministic runtime dependencies.

## ROS package boundaries

Packages may consume only installed public interfaces from other packages. `restocker_interfaces` cannot depend on application packages, and runtime packages cannot depend on `restocker_bringup`. The authoritative world state is distinct from the MoveIt planning scene.

## C++ and ROS conventions

- Use C++20 while Jazzy dependencies remain compatible.
- Prefer explicit ownership, typed status results, bounded operations, and deterministic lifetimes.
- Treat frame ID, timestamp, units, and transform freshness as part of every spatial API.
- Keep blocking I/O and model calls out of time-sensitive callbacks.
- Add SPDX license identifiers to new source files.

## Public language

Describe the project as a generic vision-guided robotic beverage-restocking platform. Do not introduce employer, customer, deployment, or proprietary-system references.
